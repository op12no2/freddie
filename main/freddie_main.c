/* firmware: Freddie the robot. AMG8833 thermal camera, LSM6DSOX IMU and
 * INA219 power monitor on one I2C bus, DRV8833 motors, and the DevKit's
 * onboard RGB status LED. Fully autonomous: the LEDs and the gestures
 * (pick up, flip, double lift-down) are his whole interface. */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AMG_SDA_GPIO    8
#define AMG_SCL_GPIO    9
#define AMG_I2C_ADDR    0x69
#define AMG_REG_PCTL    0x00   /* power control: 0x00 = normal mode */
#define AMG_REG_RST     0x01   /* reset: 0x3F = initial reset */
#define AMG_REG_FPSC    0x02   /* frame rate: 0x00 = 10 fps */
#define AMG_REG_INTC    0x03   /* interrupt control: 0x00 = disabled */
#define AMG_REG_PIXELS  0x80   /* 64 pixels x 2 bytes, little-endian */

#define INA_I2C_ADDR    0x40
#define INA_REG_CONFIG  0x00   /* 0x399F = 32 V range, ±320 mV PGA, 12-bit */
#define INA_REG_SHUNT   0x01   /* signed, 10 uV per LSB across the 0.1R shunt */
#define INA_REG_BUS     0x02   /* bits 15..3, 4 mV per LSB */

#define LSM_I2C_ADDR     0x6A
#define LSM_REG_WHOAMI   0x0F  /* reads 0x6C */
#define LSM_REG_CTRL1_XL 0x10  /* 0x40 = accel 104 Hz, ±2 g */
#define LSM_REG_CTRL2_G  0x11  /* 0x44 = gyro 104 Hz, ±500 dps */
#define LSM_REG_OUTX_L_G 0x22  /* 12 bytes: gyro xyz then accel xyz, LE */

/* Deliberately scrambled vs the DRV8833 pin names: the A channel is
 * soldered to the right motor and B to the left, and both motors have
 * inverted polarity, so we un-cross and un-invert them here. */
#define MOTOR_L_IN1_GPIO  7    /* DRV8833 BIN2 */
#define MOTOR_L_IN2_GPIO  6    /* DRV8833 BIN1 */
#define MOTOR_R_IN1_GPIO  5    /* DRV8833 AIN2 */
#define MOTOR_R_IN2_GPIO  4    /* DRV8833 AIN1 */
#define DRV_SLP_GPIO      10   /* DRV8833 nSLEEP: high = enabled */

#define PWM_FREQ_HZ     25000  /* above audible, well under DRV8833's max */
#define MOTOR_MIN_PCT   25     /* measured deadband (cal 20260804133108): no
                                  motion at 20, reliable from rest at 25;
                                  below it wheels stall-or-creep at random */
#define PWM_RES         LEDC_TIMER_10_BIT
#define PWM_MAX         (1 << 10)   /* LEDC duty range is [0, 2^res] */

#define RGB_GPIO        38     /* DevKitC-1 v1.1 onboard WS2812; v1.0 boards use 48 */

/* Status colours, dim enough to look at — the LED is blinding at full duty.
 * Red = booting or a check failed, green = all well, blue = performing.
 * Nominal green is arousal-graded: an ember asleep, full green awake
 * (George read the watch toggle as waking him, and he's right). */
#define RGB_RED         32, 0, 0
#define RGB_GREEN       0, 32, 0
#define RGB_GREEN_DIM   0, 5, 0
#define RGB_BLUE        0, 0, 48
#define RGB_BLACK       0, 0, 0
#define RGB_GLIMPSE     48, 24, 0
#define RGB_HELD        48, 0, 48
#define RGB_DREAM       0, 12, 0    /* the ember, briefly brighter */
#define DREAM_MED_S     180         /* median seconds between dreams —
                                       pure theatre, the one dishonest
                                       light he's allowed */

#define WAKE_LED_GPIO   11     /* discrete orange LED, 1 kOhm to GND, on the
                                  shelf: on = awake, off = asleep — readable
                                  in sunlight where the dim ember isn't */

#define MOTOR_L_IN1_CH  LEDC_CHANNEL_0
#define MOTOR_L_IN2_CH  LEDC_CHANNEL_1
#define MOTOR_R_IN1_CH  LEDC_CHANNEL_2
#define MOTOR_R_IN2_CH  LEDC_CHANNEL_3

static i2c_master_dev_handle_t amg, ina, lsm;
static bool amg_ok, ina_ok, lsm_ok;

static i2c_master_bus_handle_t i2c_bus;

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = AMG_SDA_GPIO,
        .scl_io_num = AMG_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));
}

static i2c_master_dev_handle_t i2c_add(uint8_t addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400 * 1000,
    };
    i2c_master_dev_handle_t dev = NULL;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev));
    return dev;
}

static esp_err_t amg_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(amg, buf, sizeof(buf), 100);
}

static void amg_init(void)
{
    amg = i2c_add(AMG_I2C_ADDR);

    /* Non-fatal: a missing sensor leaves its _ok flag false and the
     * boot LED red. */
    if (amg_write_reg(AMG_REG_PCTL, 0x00) != ESP_OK ||
        amg_write_reg(AMG_REG_RST, 0x3F) != ESP_OK ||
        amg_write_reg(AMG_REG_INTC, 0x00) != ESP_OK ||
        amg_write_reg(AMG_REG_FPSC, 0x00) != ESP_OK) {
        amg_ok = false;
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    amg_ok = true;
}

/* All 64 pixels, sign-extended raw counts (0.25 C per LSB). */
static esp_err_t amg_read_pixels(int16_t px[64])
{
    uint8_t reg = AMG_REG_PIXELS;
    uint8_t raw[128];
    esp_err_t err = i2c_master_transmit_receive(amg, &reg, 1, raw, sizeof(raw), 100);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < 64; i++) {
        int v = (raw[2 * i] | (raw[2 * i + 1] << 8)) & 0x0FFF;
        if (v & 0x800) {
            v -= 0x1000;   /* 12-bit two's complement */
        }
        px[i] = v;
    }
    return ESP_OK;
}

static esp_err_t ina_read_reg(uint8_t reg, uint16_t *val)
{
    uint8_t raw[2];
    esp_err_t err = i2c_master_transmit_receive(ina, &reg, 1, raw, sizeof(raw), 100);
    if (err != ESP_OK) {
        return err;
    }
    *val = (raw[0] << 8) | raw[1];   /* registers are big-endian */
    return ESP_OK;
}

static void ina_init(void)
{
    ina = i2c_add(INA_I2C_ADDR);
    uint8_t cfg[3] = { INA_REG_CONFIG, 0x39, 0x9F };
    if (i2c_master_transmit(ina, cfg, sizeof(cfg), 100) != ESP_OK) {
        return;
    }
    ina_ok = true;
}

/* Pack voltage in volts and current in mA, positive when discharging. */
static esp_err_t ina_read(float *volts, float *milliamps)
{
    uint16_t bus_reg, shunt_reg;
    esp_err_t err = ina_read_reg(INA_REG_BUS, &bus_reg);
    if (err == ESP_OK) {
        err = ina_read_reg(INA_REG_SHUNT, &shunt_reg);
    }
    if (err != ESP_OK) {
        return err;
    }
    *volts = (bus_reg >> 3) * 0.004f;
    *milliamps = (int16_t)shunt_reg * 0.1f;   /* 10 uV / 0.1R = 100 uA per LSB */
    return ESP_OK;
}

static void lsm_init(void)
{
    lsm = i2c_add(LSM_I2C_ADDR);
    uint8_t reg = LSM_REG_WHOAMI, id = 0;
    uint8_t xl_cfg[2] = { LSM_REG_CTRL1_XL, 0x40 };
    uint8_t g_cfg[2] = { LSM_REG_CTRL2_G, 0x44 };
    if (i2c_master_transmit_receive(lsm, &reg, 1, &id, 1, 100) != ESP_OK ||
        id != 0x6C ||
        i2c_master_transmit(lsm, xl_cfg, sizeof(xl_cfg), 100) != ESP_OK ||
        i2c_master_transmit(lsm, g_cfg, sizeof(g_cfg), 100) != ESP_OK) {
        return;
    }
    lsm_ok = true;
}

/* Gyro in degrees/s and accel in g, both x/y/z. */
static esp_err_t lsm_read(float dps[3], float g[3])
{
    uint8_t reg = LSM_REG_OUTX_L_G;
    uint8_t raw[12];
    esp_err_t err = i2c_master_transmit_receive(lsm, &reg, 1, raw, sizeof(raw), 100);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < 3; i++) {
        dps[i] = (int16_t)(raw[2 * i] | (raw[2 * i + 1] << 8)) * 0.0175f;      /* 17.5 mdps/LSB at ±500 dps */
        g[i] = (int16_t)(raw[6 + 2 * i] | (raw[7 + 2 * i] << 8)) * 0.000061f;  /* 0.061 mg/LSB at ±2 g */
    }
    return ESP_OK;
}

static rmt_channel_handle_t rgb_chan;
static rmt_encoder_handle_t rgb_enc;

static void rgb_init(void)
{
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num = RGB_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,   /* 0.1 us ticks */
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &rgb_chan));

    /* WS2812 bit timing in those ticks: 0 = 0.3 us high / 0.9 us low,
     * 1 = 0.9 us high / 0.3 us low. */
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &rgb_enc));
    ESP_ERROR_CHECK(rmt_enable(rgb_chan));

    gpio_config_t wake_cfg = {
        .pin_bit_mask = 1ULL << WAKE_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&wake_cfg));
    gpio_set_level(WAKE_LED_GPIO, 0);   /* dark until the first wake */
}

static void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t grb[3] = { g, r, b };   /* WS2812 byte order */
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    ESP_ERROR_CHECK(rmt_transmit(rgb_chan, rgb_enc, grb, sizeof(grb), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rgb_chan, 100));
    esp_rom_delay_us(60);   /* latch gap so a back-to-back frame isn't swallowed */
}

static void pwm_channel_init(ledc_channel_t ch, int gpio)
{
    ledc_channel_config_t cfg = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ch,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

static void pwm_set(ledc_channel_t ch, uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch));
}

/* pct is -100..100; 0 coasts. Slow-decay drive: the leading pin is held
 * high and the other PWM'd, which keeps torque at low speeds. */
static void motor_set(ledc_channel_t in1, ledc_channel_t in2, int pct)
{
    if (pct > 100) {
        pct = 100;
    } else if (pct < -100) {
        pct = -100;
    }
    /* Remap nonzero magnitudes onto the live range above the deadband,
     * so small commands crawl instead of gambling on breakaway. */
    int mag = abs(pct);
    if (mag > 0) {
        mag = MOTOR_MIN_PCT + mag * (100 - MOTOR_MIN_PCT) / 100;
    }
    uint32_t duty = PWM_MAX * mag / 100;
    if (pct > 0) {
        pwm_set(in1, PWM_MAX);
        pwm_set(in2, PWM_MAX - duty);
    } else if (pct < 0) {
        pwm_set(in1, PWM_MAX - duty);
        pwm_set(in2, PWM_MAX);
    } else {
        pwm_set(in1, 0);
        pwm_set(in2, 0);
    }
}

/* True when the pack is supplying current. With the pack off and USB in,
 * the 6V rail is back-fed through the DevKit's diode and the pack shunt
 * carries nothing — running motors then would pull amps through that
 * diode (see schematic.md). The pack covers the ESP32's idle draw
 * whenever it's on (measured 43-59 mA untethered, ~0 on USB), so a
 * near-zero shunt reading means USB only. */
static bool pack_live(void)
{
    float volts, ma;
    if (!ina_ok || ina_read(&volts, &ma) != ESP_OK) {
        return true;   /* can't tell — assume the pack is on */
    }
    return ma > 20.0f;
}

/* No two TT motors are matched: the trim is added to the left duty
 * (sign-aware, so it corrects magnitude in reverse too) whenever both
 * wheels are driven. Set by open-loop test, not by fitting hunt
 * logs — those commands come from steering feedback and the fit's
 * intercept is biased (it once said -6, which itself caused a left
 * veer). Measured: actual duties 49/50 drive straight. */
#define DRIVE_TRIM_PCT  -1

static int16_t cmd_left, cmd_right;   /* commanded duties pre-trim, so the
                                         hold detector can tell idle from
                                         driving */

static void drive(int left_pct, int right_pct)
{
    cmd_left = left_pct;
    cmd_right = right_pct;
    if (left_pct != 0 && right_pct != 0) {
        left_pct += (left_pct > 0) ? DRIVE_TRIM_PCT : -DRIVE_TRIM_PCT;
    }
    if (!pack_live()) {
        return;   /* USB power only: never pull motor amps through the diode */
    }
    /* The DRV8833 sleeps whenever f1 is still — which for the watcher
     * is nearly always. Wake time is well under a PWM period. */
    gpio_set_level(DRV_SLP_GPIO, left_pct != 0 || right_pct != 0);
    motor_set(MOTOR_L_IN1_CH, MOTOR_L_IN2_CH, left_pct);
    motor_set(MOTOR_R_IN1_CH, MOTOR_R_IN2_CH, right_pct);
}

static void led_nominal(void);   /* the resting green, sleep/wake graded */
static bool watch_grumpy;        /* woken too soon — see the watcher */

/* Performances hold blue for the whole act and settle to nominal on
 * exit — one convention, no colour changes mid-act (if a performance
 * ever runs atop a colour worth keeping, that's the day this becomes
 * push/pop). One function per performance until there are enough of
 * them to be worth a dispatcher. */

/* "I'm alive and all is well": wiggle on the spot. */
static void perform_alive(void)
{
    rgb_set(RGB_BLUE);
    for (int i = 0; i < 3; i++) {
        drive(60, -60);   /* above the ~30% standstill deadband */
        vTaskDelay(pdMS_TO_TICKS(100));
        drive(-60, 60);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    drive(0, 0);
    led_nominal();
}

/* "Hello": the greeting when the watcher glimpses warmth. */
static void perform_hello(void)
{
    rgb_set(RGB_BLUE);
    for (int i = 0; i < 1; i++) {
        drive(40, -40);   /* above the ~30% standstill deadband */
        vTaskDelay(pdMS_TO_TICKS(200));
        drive(-40, 40);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    drive(0, 0);
    led_nominal();
}

/* "I'm awake": a lazy stretch — lean forward, hold it, settle back.
 * Runs at every wake; it may nose an obstacle, and that's fine.
 * Woken too soon, the lean drags (slower, so a little shorter) and
 * the hold lasts twice as long. */
static void perform_awake(void)
{
    int duty = watch_grumpy ? 28 : 35;   /* both above the deadband */
    rgb_set(RGB_BLUE);
    drive(duty, duty);
    vTaskDelay(pdMS_TO_TICKS(500));
    drive(0, 0);
    vTaskDelay(pdMS_TO_TICKS(watch_grumpy ? 2000 : 1000));
    drive(-duty, -duty);
    vTaskDelay(pdMS_TO_TICKS(500));
    drive(0, 0);
    led_nominal();
}

/* "Found you!": a quick excited shimmy, used when the hunt spots heat. */
static void perform_found(void)
{
    rgb_set(RGB_BLUE);
    for (int i = 0; i < 2; i++) {
        drive(40, -40);
        vTaskDelay(pdMS_TO_TICKS(120));
        drive(-40, 40);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    drive(0, 0);
    led_nominal();
}

static void motors_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    pwm_channel_init(MOTOR_L_IN1_CH, MOTOR_L_IN1_GPIO);
    pwm_channel_init(MOTOR_L_IN2_CH, MOTOR_L_IN2_GPIO);
    pwm_channel_init(MOTOR_R_IN1_CH, MOTOR_R_IN1_GPIO);
    pwm_channel_init(MOTOR_R_IN2_CH, MOTOR_R_IN2_GPIO);

    gpio_config_t slp_cfg = {
        .pin_bit_mask = 1ULL << DRV_SLP_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&slp_cfg));
    gpio_set_level(DRV_SLP_GPIO, 0);   /* asleep until the first move */
}

#define TICK_HZ 10   /* the sensor/behaviour heartbeat */

/* Pickup detection: handling is unmistakable to the gyro — 150 dps
 * against a sub-1 dps floor when he's parked (wall-run log). Watched
 * only while the motors are idle, which for the watcher is nearly
 * always. Violet LED while held. */
#define HELD_DPS      20.0f   /* sustained rotation while idle = in hands */
#define HELD_DRIVE_AZ 0.90f   /* sustained tilt (~25 deg) while driving =
                                 lifted; the habitat's floors are flat */
#define HELD_ON_TICKS 3       /* a knock is one tick; a carry is many */
#define HELD_QUIET_DPS 5.0f
#define HELD_OFF_MS   1000    /* this long quiet again = set down */

/* Gesture command, Foundation-style: two brief lift-downs in quick
 * succession toggle the watcher — the only switch he has. */
#define GEST_LIFT_MAX_S 4     /* each lift-down this brief (incl. quiet 1 s) */
#define GEST_GAP_MAX_S  3     /* and the next lift this soon after set-down */

/* The sprint's arming gesture rides on held: fully inverted for a
 * second, then set down. Nobody flips a robot by accident — the
 * safest trigger the IMU can offer for the fastest thing he does. */
#define FLIP_AZ       -0.8f   /* upside down, unmistakably */
#define FLIP_TICKS    10      /* a full second of it */

static bool held;
static int held_ticks;
static int64_t held_quiet_since;
static int64_t held_since, gest_setdown_at;
static int gest_count;
static volatile bool gest_toggle;   /* double lift-down seen: flip the watcher */
static int flip_ticks;
static bool sprint_armed;
static volatile bool sprint_go;     /* flip-armed set-down seen: sprint */
static volatile bool knock_felt;    /* a rejected knock: too brief to be
                                       hands, but he felt it */

static void held_check(const float dps[3], const float g[3])
{
    float gmag = sqrtf(dps[0] * dps[0] + dps[1] * dps[1] + dps[2] * dps[2]);
    int64_t now = esp_timer_get_time();
    if (!held) {
        /* Idle, rotation betrays hands; driving, rotation is normal and
         * tilt is the witness instead. */
        bool idle = cmd_left == 0 && cmd_right == 0;
        bool in_hands = idle ? gmag > HELD_DPS : g[2] < HELD_DRIVE_AZ;
        if (idle && !in_hands && held_ticks > 0) {
            knock_felt = true;   /* spiked but didn't become a hold */
        }
        held_ticks = in_hands ? held_ticks + 1 : 0;
        if (held_ticks >= HELD_ON_TICKS) {
            held = true;
            held_quiet_since = 0;
            if (now - gest_setdown_at > GEST_GAP_MAX_S * 1000000LL) {
                gest_count = 0;   /* too slow, the ritual starts over */
            }
            held_since = now;
            flip_ticks = 0;
            sprint_armed = false;   /* each hold is a fresh ritual */
            drive(0, 0);   /* wheels stop, and the driver sleeps, in hands */
            rgb_set(RGB_HELD);   /* violet: airborne */
        }
    } else if (g[2] < FLIP_AZ) {
        if (++flip_ticks >= FLIP_TICKS && !sprint_armed) {
            sprint_armed = true;
            rgb_set(RGB_BLUE);   /* blue in hand: armed */
        }
        held_quiet_since = 0;   /* inverted stillness is not a set-down */
    } else if (gmag < HELD_QUIET_DPS && g[2] > HELD_DRIVE_AZ) {
        /* quiet and upright — only that means back on his wheels */
        flip_ticks = 0;   /* the arming second must be contiguous */
        if (held_quiet_since == 0) {
            held_quiet_since = now;
        } else if (now - held_quiet_since > HELD_OFF_MS * 1000) {
            held = false;
            held_ticks = 0;
            led_nominal();
            if (sprint_armed) {
                sprint_armed = false;
                gest_count = 0;        /* the flip spends this lift-down */
                sprint_go = true;
            } else if (now - held_since < GEST_LIFT_MAX_S * 1000000LL) {
                if (++gest_count >= 2) {
                    gest_count = 0;
                    gest_toggle = true;
                }
            } else {
                gest_count = 0;
            }
            gest_setdown_at = now;
        }
    } else {
        held_quiet_since = 0;
        flip_ticks = 0;
    }
}

/* Watcher: the resting heartbeat. Rest with the driver asleep, learning
 * a per-pixel background; look around (a gyro-metered full circle) at
 * log-normally random intervals whose median stretches as the pack
 * tires; a glimpse of warmth earns a beat of amber and a hello wiggle
 * and pulls the next look closer (a set-down pulls it too); the sweep
 * itself slows when something warm crosses the view — the gaze lingers
 * — and ends by turning back, shortest way round, to the warmest
 * heading it saw, or to the glimpse's angle if the sweep found nothing,
 * or nowhere at all if there was neither. All knobs below; thresholds
 * cite f1/log/. */
#define WATCH_LOOK_MED_S    180     /* median rest between looks, fresh pack */
#define WATCH_LOOK_SIGMA    0.7f    /* log-normal spread: double-takes and naps */
#define WATCH_LOOK_MIN_S    30
#define WATCH_LOOK_MAX_S    1800
#define WATCH_FRESH_V       5.4f    /* resting volts at mood 0 (fresh) */
#define WATCH_TIRED_V       4.7f    /* resting volts at mood 1 (tired) */
#define WATCH_TIRED_SCALE   3.0f    /* median multiplier when fully tired */
#define WATCH_VREST_ALPHA   0.01f   /* resting-voltage EMA speed */
#define WATCH_SOON_S        20      /* a nudge pulls the next look to about
                                       here — jittered +-half, so noticing
                                       never runs on a timetable */
#define WATCH_AWAKE_MED_S   7200    /* median awake span, fresh — tiredness
                                       shrinks it (/3 fully tired) */
#define WATCH_SLEEP_MED_S   3600    /* median sleep span, fresh — tiredness
                                       stretches it (x3 fully tired) */
#define WATCH_CYCLE_SIGMA   0.5f    /* tighter than the looks: a rhythm,
                                       not a lottery */
#define WATCH_AWAKE_MIN_S   1200    /* 20 min .. 6 h awake */
#define WATCH_AWAKE_MAX_S   21600
#define WATCH_SLEEP_MIN_S   900     /* 15 min .. 4 h asleep */
#define WATCH_SLEEP_MAX_S   14400
#define WATCH_SPIN_PCT      20      /* sweep duty (pre-remap), fresh */
#define WATCH_SPIN_TIRED_PCT 8      /* sweep duty when fully tired */
#define WATCH_SPIN_MIN_PCT  1       /* gaze-drag floor: linger, never stall */
#define WATCH_GAZE_K        8.0f    /* duty shed per C of passing warmth */
#define WATCH_GAZE_DEAD_C   0.8f    /* scene contrast to ignore (empty-room
                                       max-med runs ~0.9-1.4, gestures.log) */
#define WATCH_TURN_DEG      360.0f
#define WATCH_TURN_TIMEOUT_S 30
#define WATCH_GLIMPSE_C     1.5f    /* px over background = something's there
                                       (zero false alarms, quiet_room_sat) */
#define WATCH_GLIMPSE_BEAT_MS 300     /* amber beat of noticing before the hello */
#define WATCH_BG_ALPHA      0.02f   /* per-pixel background EMA */
#define WATCH_BG_SETTLE_S   10      /* stillness before the background is
                                       trusted (quiet_room_sat) */
#define WATCH_CENTER_COL    3.2f    /* boresight column (measured) */
#define WATCH_COL_DEG       7.5f    /* camera columns to degrees */
#define WATCH_SHRUG_FRESH   0.15f   /* odds the first glimpse's hello is
                                       withheld anyway — greeting is
                                       never a certainty */
#define WATCH_SHRUG_TIRED   0.60f   /* ...on a flat pack: mostly can't
                                       be bothered */
#define WATCH_GRUMPY_FRAC   0.25f   /* woken in the first quarter of a
                                       sleep = woken too soon */
#define WATCH_SHRUG_GRUMPY  0.90f   /* grumpy shrug odds, first rest only —
                                       the first look walks it off */
#define WATCH_REORIENT_MIN_DEG 5.0f /* not worth turning back for less */
#define WATCH_REORIENT_TIMEOUT_S 15
#define WATCH_WIND_MAX_S    2700    /* second wind: total extra awake time
                                       the day's events can earn, fresh —
                                       tiredness shrinks it (/3 fully tired) */
#define WATCH_WIND_FRAC     0.3f    /* each event takes this fraction of the
                                       pot's remainder: geometric, so the
                                       first glimpse of the evening matters
                                       most and a busy room can't run away */
#define WATCH_DOZE_MAX_S    1800    /* nothing doing: total awake time boring
                                       looks can dock, fresh — tiredness
                                       stretches it (x3 fully tired) */
#define WATCH_DOZE_FRAC     0.2f

/* The coax: when a look ends facing something warm, f1 may come partway
 * to meet it — carefully, after checking his facts, and never all the
 * way. Every threshold below is a measured number (gestures.log,
 * quiet_room_sat.log); the doctrine is the watcher's: sense only at
 * rest, move blind and short, end in stillness. */
#define COAX_BLOB_C      2.0f  /* px over frame mean = part of the visitor:
                                  a standing person at 2 m clears the frame
                                  median by +2.0-2.4 C (gestures.log) */
#define COAX_MIN_PX      2     /* minimum blob weight — one noise pixel
                                  (±2.5 C) can't be a visitor */
#define COAX_DWELL_S     5     /* dwell >= ~5 s separates visit from
                                  transit with no overlap (gestures.log) */
#define COAX_PRESENCE    0.6f  /* real coaxing sustained 76-80% presence */
#define COAX_LIVELY      0.02f /* centroid-motion EMA: people 0.03-0.07, a
                                  hot window 0.007 — movement, not
                                  temperature, tells people from furniture */
#define COAX_CEIL_PX     14    /* close enough: 1 m from a crouching child
                                  fills 14 px (gestures.log). Never fill
                                  the frame — full-frame = possible bump */
#define COAX_FULL_PX     32    /* half the frame = someone on top: freeze */
#define COAX_RETREAT_PX  2.0f  /* the blob shrank = backing away (retreat
                                  is a clean monotonic ramp) — a shy
                                  creature does not chase */
#define COAX_HOPS_MAX    3     /* he comes partway; they meet him in the
                                  middle */
#define COAX_HOP_PCT     30
#define COAX_HOP_MS      1200  /* one tentative hop, shrinking as the blob
                                  grows so he creeps as he nears */
#define COAX_HOP_MIN_MS  500
#define COAX_TURN_PCT    20
#define COAX_TURN_MIN_DEG 6.0f /* under a pixel column — not worth turning */
#define COAX_TURN_TIMEOUT_S 5
#define COAX_OBS_SETTLE_MS 500 /* let the stop settle before re-observing */
#define COAX_OBS_S       2     /* the re-observation window after a hop */
#define COAX_NERVE_MAX   3     /* rolls per hop: up to two visible false
                                  starts, then he either goes or gives up */
#define COAX_NERVE_GAP_MS 1500
#define COAX_NERVE_BOOST 0.25f /* each failed roll adds this — the nerve
                                  visibly builds */
#define COAX_P_MAX       0.85f /* never a certainty, like the hello */

typedef enum { WATCH_OFF = 0, WATCH_REST, WATCH_LOOK, WATCH_ORIENT,
               WATCH_DWELL, WATCH_NERVE, WATCH_CTURN, WATCH_HOP,
               WATCH_OBS } watch_state_t;

static watch_state_t watch_state;
static float watch_bg[64];
static bool watch_bg_seed = true;
static int64_t watch_bg_ok_at;     /* background trusted after this */
static int64_t watch_deadline;     /* next scheduled look */
static int64_t watch_look_until;   /* spin safety timeout */
static float watch_yaw;
static int watch_sign;
static float watch_vrest;          /* resting pack volts, slow EMA */
static bool watch_prev_held;
static bool watch_glimpse_prev;
static bool watch_hello_done;      /* one hello per rest: stay surprising */
static float watch_reorient_deg;   /* remaining degrees of the settle turn */
static float watch_best_drag;      /* warmest moment of the sweep... */
static float watch_best_yaw;       /* ...and the yaw it was seen at */
static bool watch_glimpse_pending;   /* a glimpse called this look */
static int64_t watch_cycle_at;     /* next autonomous sleep/wake toggle */
static int64_t watch_woke_at;      /* start of this awake span */
static int64_t watch_slept_at;     /* start of this sleep span */
static bool watch_huh;             /* a glimpse called the look, the look
                                       found nothing: puzzled settle */
static float watch_wind_s;         /* second-wind pot left this span */
static float watch_doze_s;         /* nothing-doing pot left this span */
static bool watch_duty;            /* the rhythm: armed by the first wake,
                                      deep sleep from power-on until then */
static int watch_glimpse_sign;       /* drive sign toward the last glimpse */
static float watch_glimpse_deg;      /* its degrees off boresight */

static int64_t coax_until;         /* current coax phase deadline */
static int64_t coax_obs_from;      /* observation windows discard frames
                                      before this (post-hop settle) */
static int coax_ticks, coax_seen;  /* window frames, and frames with a blob */
static float coax_px_sum;          /* blob weight summed over the window */
static float coax_col;             /* last-seen blob centroid column */
static float coax_lively;          /* centroid-motion EMA over the window */
static float coax_prev_col, coax_prev_row;
static bool coax_prev_seen;
static bool coax_full;             /* the blob hit COAX_FULL_PX: freeze */
static float coax_prev_px;         /* blob weight before the last hop */
static int coax_hops, coax_nerve;
static float coax_p;               /* this hop's base odds */
static float coax_turn_deg;

/* The resting green, graded by arousal: a dim ember while the watcher
 * sleeps, full green awake — and the discrete orange wake LED agrees,
 * on awake, off asleep, for sunlight the ember can't fight. Every
 * "settle back to normal" goes through here so sleep and wakefulness
 * read at a glance. */
static void led_nominal(void)
{
    gpio_set_level(WAKE_LED_GPIO, watch_state != WATCH_OFF);
    if (watch_state == WATCH_OFF) {
        rgb_set(RGB_GREEN_DIM);
    } else {
        rgb_set(RGB_GREEN);
    }
}

/* The sprint — "how fast can he go?", George's question answered with
 * theatre. Five countdown winks to aim him and stand clear (a safety
 * poll under each), a full-duty second out, a fast gyro-metered
 * about-face, and a second home, easing off the throttle over each
 * leg's tail so he pulls up rather than skids. Armed by the flip
 * gesture. Floor doctrine: the fuse cannot know he's on a table.
 * Aborts on handling, tilt, or unexpected rotation (which a
 * full-speed wall hit becomes). */
#define SPRINT_FUSE_S    5
#define SPRINT_RUN_PCT   100
#define SPRINT_RUN_MS    2300
#define SPRINT_DECEL_MS  300     /* throttle ramps out over the leg's tail */
#define SPRINT_BACK_MS   2250    /* tune so the return ends near the start */
#define SPRINT_SPIN_PCT  60      /* dramatic, but under the gyro's ±500 dps —
                                    a saturated meter over-rotates the 180 */
#define SPRINT_TURN_DEG  180.0f
#define SPRINT_SPIN_TIMEOUT_MS 3000
#define SPRINT_GUARD_DPS 60.0f   /* yaw he never shows driving straight */
#define SPRINT_BREATHER_S 75     /* catching his breath: the next look
                                    waits about this long (the launch
                                    genuinely sagged the pack) */
#define SPRINT_BAD_TICKS 12      /* ~120 ms sustained before a moving abort:
                                    launch vibration (p99 0.24 g at 10 Hz)
                                    spikes across the tilt line, a lift
                                    stays across it */

static int sprint_bad;

/* One safety poll: handling, tilt, unexpected yaw (max_dps 0 =
 * spinning on purpose, don't judge). Tilt/yaw must persist bad_ticks
 * consecutive polls — 1 while stationary (the fuse), SPRINT_BAD_TICKS
 * while moving. */
static bool sprint_safe(float max_dps, int bad_ticks, float *gz)
{
    float dps[3], g[3];
    if (held) {
        return false;
    }
    if (lsm_read(dps, g) != ESP_OK) {
        return false;
    }
    bool bad = g[2] < HELD_DRIVE_AZ ||               /* lifted or falling */
               (max_dps > 0 && fabsf(dps[2]) > max_dps);   /* turned */
    sprint_bad = bad ? sprint_bad + 1 : 0;
    if (gz) {
        *gz = dps[2];
    }
    return sprint_bad < bad_ticks;
}

static bool sprint_leg(int ms)
{
    int64_t t0 = esp_timer_get_time();
    int el;
    while ((el = (int)((esp_timer_get_time() - t0) / 1000)) < ms) {
        int pct = SPRINT_RUN_PCT;
        if (ms - el < SPRINT_DECEL_MS) {
            pct = SPRINT_RUN_PCT * (ms - el) / SPRINT_DECEL_MS;
        }
        drive(pct, pct);
        if (!sprint_safe(SPRINT_GUARD_DPS, SPRINT_BAD_TICKS, NULL)) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

static bool sprint_spin(void)
{
    float yaw = 0, gz;
    int64_t t0 = esp_timer_get_time(), last = t0;
    drive(SPRINT_SPIN_PCT, -SPRINT_SPIN_PCT);
    while (fabsf(yaw) < SPRINT_TURN_DEG) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!sprint_safe(0, SPRINT_BAD_TICKS, &gz)) {
            return false;
        }
        int64_t now = esp_timer_get_time();
        yaw += gz * (float)(now - last) / 1000000.0f;
        last = now;
        if (now - t0 > SPRINT_SPIN_TIMEOUT_MS * 1000LL) {
            break;   /* gyro trouble: don't pirouette forever */
        }
    }
    return true;
}

static void perform_sprint(void)
{
    if (!lsm_ok) {
        return;   /* no IMU, no sprint */
    }
    if (watch_state == WATCH_REST) {
        /* no look mid-sprint */
        int64_t busy = esp_timer_get_time() + 15 * 1000000LL;
        if (watch_deadline < busy) {
            watch_deadline = busy;
        }
    }
    sprint_bad = 0;
    bool ok = true;
    for (int i = 0; ok && i < SPRINT_FUSE_S; i++) {
        rgb_set(RGB_BLUE);
        for (int j = 0; ok && j < 20; j++) {
            ok = sprint_safe(HELD_DPS, 1, NULL);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        rgb_set(RGB_BLACK);
        for (int j = 0; ok && j < 80; j++) {
            ok = sprint_safe(HELD_DPS, 1, NULL);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    /* distance varies hop to hop: anywhere from the usual leg out to
     * twice that, so the sprint doesn't always cover the same ground */
    float dist_mult = 1.0f + (esp_random() % 1001) / 1000.0f;   /* [1.0, 2.0] */
    int run_ms = (int)(SPRINT_RUN_MS * dist_mult);
    int back_ms = (int)(SPRINT_BACK_MS * dist_mult);
    rgb_set(RGB_BLUE);
    ok = ok && sprint_leg(run_ms);
    ok = ok && sprint_spin();
    ok = ok && sprint_leg(back_ms);
    drive(0, 0);
    watch_bg_seed = true;   /* wherever he ended up, the eye moved */
    led_nominal();
    if (ok && watch_state == WATCH_REST) {
        /* the breather: that genuinely cost him — the next look waits */
        int br = SPRINT_BREATHER_S / 2 +
                 (int)(esp_random() % (SPRINT_BREATHER_S + 1));
        watch_deadline = esp_timer_get_time() + (int64_t)br * 1000000LL;
    }
}

static float watch_frand(void)
{
    return ((esp_random() >> 8) + 0.5f) / 16777216.0f;   /* (0,1) */
}

/* "Soon", as a creature means it: WATCH_SOON_S +- half. */
static int64_t watch_soon_us(void)
{
    return (int64_t)(WATCH_SOON_S * (0.5f + watch_frand()) * 1000000.0f);
}

/* 0 = fresh pack, 1 = tired, from the resting-voltage EMA. */
static float watch_mood(void)
{
    if (watch_vrest == 0) {
        return 0;
    }
    float m = (WATCH_FRESH_V - watch_vrest) / (WATCH_FRESH_V - WATCH_TIRED_V);
    return m < 0 ? 0 : m > 1 ? 1 : m;
}

/* Log-normal draw: how creatures keep appointments. */
static float watch_lognormal_s(float med, float sigma, float min_s,
                               float max_s)
{
    float z = sqrtf(-2.0f * logf(watch_frand())) *
              cosf(6.2831853f * watch_frand());
    float s = med * expf(sigma * z);
    return s < min_s ? min_s : s > max_s ? max_s : s;
}

/* 1 fresh .. WATCH_TIRED_SCALE fully tired. */
static float watch_tired(void)
{
    return 1.0f + watch_mood() * (WATCH_TIRED_SCALE - 1.0f);
}

/* Rest interval between looks, median stretched by mood. */
static float watch_draw_s(void)
{
    return watch_lognormal_s(WATCH_LOOK_MED_S * watch_tired(),
                             WATCH_LOOK_SIGMA,
                             WATCH_LOOK_MIN_S, WATCH_LOOK_MAX_S);
}

/* The day's content gets a vote on bedtime. Interesting events — a
 * glimpse at rest, a gaze the look confirmed — spend the second-wind pot
 * pushing sleep later; a look that found nothing at all spends the
 * nothing-doing pot pulling it closer. Each takes a fraction of what's
 * left in its pot, so the first event of the evening matters most and
 * neither direction can run away; the deadline stays inside the usual
 * awake clamps, measured from wake. */
static void watch_bedtime_nudge(bool interesting)
{
    float take;
    if (interesting) {
        take = watch_wind_s * WATCH_WIND_FRAC;
        watch_wind_s -= take;
    } else {
        take = -(watch_doze_s * WATCH_DOZE_FRAC);
        watch_doze_s += take;
    }
    int64_t at = watch_cycle_at + (int64_t)(take * 1000000.0f);
    int64_t lo = watch_woke_at + WATCH_AWAKE_MIN_S * 1000000LL;
    int64_t hi = watch_woke_at + WATCH_AWAKE_MAX_S * 1000000LL;
    watch_cycle_at = at < lo ? lo : at > hi ? hi : at;
}

/* Shared by the sleep/wake rhythm and the double lift-down gesture. The
 * winks are the acknowledgment: blue-blue = watching, red = not. */
static void watch_toggle(void)
{
    if (watch_state != WATCH_OFF) {
        watch_state = WATCH_OFF;
        drive(0, 0);
        float span = watch_lognormal_s(WATCH_SLEEP_MED_S * watch_tired(),
                                       WATCH_CYCLE_SIGMA,
                                       WATCH_SLEEP_MIN_S, WATCH_SLEEP_MAX_S);
        watch_slept_at = esp_timer_get_time();
        watch_cycle_at = watch_slept_at + (int64_t)(span * 1000000.0f);
        rgb_set(RGB_RED);
        vTaskDelay(pdMS_TO_TICKS(800));
        led_nominal();   /* back to sleep: the ember */
    } else if (amg_ok) {   /* no thermal camera, no watcher */
        for (int i = 0; i < 2; i++) {
            rgb_set(RGB_BLUE);
            vTaskDelay(pdMS_TO_TICKS(400));
            rgb_set(RGB_BLACK);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        watch_bg_seed = true;
        watch_prev_held = false;
        watch_glimpse_pending = false;
        watch_hello_done = false;
        knock_felt = false;   /* pokes from before the wake don't count */
        /* woken in the first quarter of the sleep he'd drawn = woken
         * too soon: the stretch drags and the first hello is unlikely,
         * until the first look walks it off */
        watch_grumpy = watch_slept_at > 0 && watch_cycle_at > watch_slept_at &&
                       esp_timer_get_time() - watch_slept_at <
                       (int64_t)((watch_cycle_at - watch_slept_at) *
                                 WATCH_GRUMPY_FRAC);
        watch_deadline = esp_timer_get_time() + watch_soon_us();
        watch_duty = true;   /* the rhythm starts with the first wake */
        float span = watch_lognormal_s(WATCH_AWAKE_MED_S / watch_tired(),
                                       WATCH_CYCLE_SIGMA,
                                       WATCH_AWAKE_MIN_S, WATCH_AWAKE_MAX_S);
        watch_woke_at = esp_timer_get_time();
        watch_cycle_at = watch_woke_at + (int64_t)(span * 1000000.0f);
        /* the bedtime pots, tiredness-asymmetric like the spans: a tired
         * f1 is harder to keep up and quicker to give up on a dead room */
        watch_wind_s = WATCH_WIND_MAX_S / watch_tired();
        watch_doze_s = WATCH_DOZE_MAX_S * watch_tired();
        watch_state = WATCH_REST;
        perform_awake();   /* the waking stretch; ends on full green */
    }
}

/* The look is over: stop, reseed the (now stale) background, draw the
 * next rest. Shared by the sweep's end and the settle turn's end. */
static void watch_settle(int64_t now)
{
    drive(0, 0);
    if (watch_huh) {
        /* the glimpse promised something and the circle found nothing:
         * one small head-shake at the promised angle — I could have
         * sworn... */
        watch_huh = false;
        vTaskDelay(pdMS_TO_TICKS(150));
        drive(30, -30);
        vTaskDelay(pdMS_TO_TICKS(80));
        drive(-30, 30);
        vTaskDelay(pdMS_TO_TICKS(80));
        drive(0, 0);
    }
    led_nominal();
    watch_bg_seed = true;
    float rest = watch_draw_s();
    watch_deadline = now + (int64_t)(rest * 1000000.0f);
    watch_state = WATCH_REST;
}

/* The coaxing eye: a blob against the frame mean, not the per-pixel
 * background — the background needs 10 s of stillness the coax never
 * has, and at coaxing range a person clears the frame mean standing
 * (gestures.log). Clean division of labour: per-pixel background = the
 * sensitive rest-state glimpse detector, frame-mean blob = the robust
 * close-range coax eye. Returns the blob weight in pixels; centroid in
 * image coordinates when there is one. */
static int coax_blob(const float t[64], float mean, float *col, float *row)
{
    int n = 0;
    float csum = 0, rsum = 0;
    for (int i = 0; i < 64; i++) {
        if (t[i] - mean >= COAX_BLOB_C) {
            n++;
            csum += i % 8;
            rsum += i / 8;
        }
    }
    if (n < COAX_MIN_PX) {
        return 0;
    }
    *col = csum / n;
    *row = rsum / n;
    return n;
}

/* Open a fresh observation window: frames before the settle are
 * discarded, then presence, size and liveliness accumulate. */
static void coax_window(int64_t now, int settle_ms, int span_s)
{
    coax_obs_from = now + settle_ms * 1000LL;
    coax_until = coax_obs_from + span_s * 1000000LL;
    coax_ticks = coax_seen = 0;
    coax_px_sum = 0;
    coax_lively = 0;
    coax_prev_seen = false;
    coax_full = false;
}

/* One frame of blob accounting, shared by the dwell and the
 * re-observation after each hop. */
static void coax_accumulate(const float t[64], float mean, int64_t now)
{
    if (now < coax_obs_from) {
        return;
    }
    float col, row;
    int n = coax_blob(t, mean, &col, &row);
    coax_ticks++;
    if (n > 0) {
        coax_seen++;
        coax_px_sum += n;
        if (n >= COAX_FULL_PX) {
            coax_full = true;
        }
        if (coax_prev_seen) {
            float move = fabsf(col - coax_prev_col) +
                         fabsf(row - coax_prev_row);
            coax_lively += 0.2f * (move - coax_lively);
        }
        coax_prev_col = col;
        coax_prev_row = row;
        coax_col = col;
        coax_prev_seen = true;
    } else {
        coax_prev_seen = false;
    }
}

/* Working up the nerve, visibly: a little lean forward, a frozen beat,
 * and back down. The roll failed, and everyone watching knows exactly
 * what almost happened — which is the point: it reads as shyness and
 * invites more coaxing. */
static void coax_false_start(void)
{
    rgb_set(RGB_BLUE);
    drive(35, 35);
    vTaskDelay(pdMS_TO_TICKS(150));
    drive(0, 0);
    vTaskDelay(pdMS_TO_TICKS(400));
    drive(-35, -35);
    vTaskDelay(pdMS_TO_TICKS(150));
    drive(0, 0);
    led_nominal();
}

/* Set up the dice for the next hop. Odds ∝ presence × blob size
 * (gestures.log: response probability ∝ blob size × dwell falls
 * straight out), damped by tiredness, capped short of certainty — a
 * child coaxing harder genuinely raises them, but he's never a
 * machine that always comes. */
static void coax_consider(int64_t now, float presence, float px)
{
    coax_prev_px = px;
    coax_p = presence * (px / COAX_CEIL_PX) * (1.0f - 0.5f * watch_mood());
    if (coax_p > COAX_P_MAX) {
        coax_p = COAX_P_MAX;
    }
    coax_nerve = 0;
    coax_until = now + COAX_NERVE_GAP_MS * 1000LL;
    watch_state = WATCH_NERVE;
}

/* The hop: short, blind, gentle, shrinking as the blob grows so he
 * creeps as he nears. The tilt witness covers edges and climbs, as it
 * does for every move. */
static void coax_hop_start(int64_t now)
{
    int ms = (int)(COAX_HOP_MS * (1.0f - coax_prev_px / COAX_CEIL_PX));
    if (ms < COAX_HOP_MIN_MS) {
        ms = COAX_HOP_MIN_MS;
    }
    rgb_set(RGB_BLUE);
    drive(COAX_HOP_PCT, COAX_HOP_PCT);
    coax_until = now + ms * 1000LL;
    watch_state = WATCH_HOP;
}

/* The look is over: if it ended facing warmth, check the facts before
 * anything else — sit still and watch. Otherwise rest as ever. */
static void watch_look_done(int64_t now)
{
    if (watch_best_drag > 0 && !watch_huh) {
        drive(0, 0);
        led_nominal();
        coax_hops = 0;
        coax_window(now, 0, COAX_DWELL_S);
        watch_state = WATCH_DWELL;
        return;
    }
    watch_settle(now);
}

static void watch_step(const int16_t px[64], const float dps[3],
                       float volts, bool volts_ok)
{
    int64_t now = esp_timer_get_time();
    if (volts_ok && cmd_left == 0 && cmd_right == 0 && volts > 3.0f) {
        watch_vrest = (watch_vrest == 0)
            ? volts : watch_vrest + WATCH_VREST_ALPHA * (volts - watch_vrest);
    }
    if (held) {
        watch_state = WATCH_REST;   /* go limp; motors already stopped */
        watch_prev_held = true;
        return;
    }
    if (watch_prev_held) {
        watch_prev_held = false;
        watch_bg_seed = true;       /* new spot, new background */
        watch_glimpse_pending = false;
        watch_hello_done = false;
        watch_deadline = now + watch_soon_us();
    }

    float t[64];
    float maxt = -100, sum = 0;
    for (int i = 0; i < 64; i++) {
        t[i] = px[i] * 0.25f;
        sum += t[i];
        if (t[i] > maxt) {
            maxt = t[i];
        }
    }
    float mean = sum / 64;

    switch (watch_state) {
    case WATCH_REST: {
        if (knock_felt) {
            /* The flinch: a knock the pickup detector rejected still
             * deserves a startle — recoil, a beat of amber, and a look
             * soon ("what was that?"). Being poked keeps you awake. */
            knock_felt = false;
            rgb_set(RGB_GLIMPSE);
            drive(-30, -30);
            vTaskDelay(pdMS_TO_TICKS(120));
            drive(0, 0);
            vTaskDelay(pdMS_TO_TICKS(WATCH_GLIMPSE_BEAT_MS));
            led_nominal();
            watch_bg_seed = true;   /* the recoil moved the eye */
            watch_bedtime_nudge(true);
            int64_t soon = watch_soon_us();
            if (watch_deadline > now + soon) {
                watch_deadline = now + soon;
            }
            break;
        }
        if (watch_bg_seed) {
            memcpy(watch_bg, t, sizeof(watch_bg));
            watch_bg_ok_at = now + WATCH_BG_SETTLE_S * 1000000LL;
            watch_bg_seed = false;
        }
        float maxdev = -100;
        int maxdev_i = 0;
        for (int i = 0; i < 64; i++) {
            float dev = t[i] - watch_bg[i];
            if (dev > maxdev) {
                maxdev = dev;
                maxdev_i = i;
            }
        }
        bool glimpse = maxdev >= WATCH_GLIMPSE_C && now > watch_bg_ok_at;
        if (maxdev < WATCH_GLIMPSE_C) {
            /* learn only quiet frames, so a visitor can't become wall */
            for (int i = 0; i < 64; i++) {
                watch_bg[i] += WATCH_BG_ALPHA * (t[i] - watch_bg[i]);
            }
        }
        if (glimpse && !watch_glimpse_prev) {
            /* a beat of amber ("interesting"), and the warmth's angle
             * is remembered for the look's settle — but the hello only
             * once per rest, or he's very predictable */
            float off = (maxdev_i % 8) - WATCH_CENTER_COL;
            /* sign field-tested: he turned away from the first tester —
             * image columns run mirrored to the guess */
            watch_glimpse_sign = off > 0 ? -1 : 1;
            watch_glimpse_deg = fabsf(off) * WATCH_COL_DEG;
            watch_glimpse_pending = true;
            if (!watch_hello_done &&
                watch_frand() <
                (watch_grumpy ? WATCH_SHRUG_GRUMPY
                              : WATCH_SHRUG_FRESH + watch_mood() *
                                (WATCH_SHRUG_TIRED - WATCH_SHRUG_FRESH))) {
                watch_hello_done = true;   /* can't be bothered: the
                                              shrug spends the hello */
            }
            watch_bedtime_nudge(true);   /* someone appeared */
            rgb_set(RGB_GLIMPSE);
            vTaskDelay(pdMS_TO_TICKS(WATCH_GLIMPSE_BEAT_MS));
            if (!watch_hello_done) {
                watch_hello_done = true;
                perform_hello();        /* ends on green */
                watch_bg_seed = true;   /* the wiggle moved the eye a little */
            } else {
                led_nominal();          /* noticed, said nothing */
            }
            int64_t soon = watch_soon_us();
            if (watch_deadline > now + soon) {
                watch_deadline = now + soon;
            }
        }
        watch_glimpse_prev = glimpse;
        if (now >= watch_deadline) {
            watch_yaw = 0;
            watch_sign = (esp_random() & 1) ? 1 : -1;
            watch_best_drag = 0;
            watch_hello_done = false;   /* the look re-arms the hello */
            watch_grumpy = false;       /* and walks off the grump */
            watch_look_until = now + WATCH_TURN_TIMEOUT_S * 1000000LL;
            watch_state = WATCH_LOOK;
            rgb_set(RGB_BLUE);
        }
        break;
    }
    case WATCH_LOOK: {
        /* the gaze lingers: passing warmth sheds sweep duty */
        float drag = (maxt - mean) - WATCH_GAZE_DEAD_C;
        if (drag < 0) {
            drag = 0;
        }
        if (drag > watch_best_drag) {   /* the warmest heading so far */
            watch_best_drag = drag;
            watch_best_yaw = watch_yaw;
        }
        int base = WATCH_SPIN_PCT -
                   (int)(watch_mood() * (WATCH_SPIN_PCT - WATCH_SPIN_TIRED_PCT));
        int duty = base - (int)(WATCH_GAZE_K * drag);
        if (duty < WATCH_SPIN_MIN_PCT) {
            duty = WATCH_SPIN_MIN_PCT;
        }
        drive(watch_sign * duty, -watch_sign * duty);
        watch_yaw += dps[2] * (1.0f / TICK_HZ);
        if (fabsf(watch_yaw) >= WATCH_TURN_DEG || now > watch_look_until) {
            /* circle done: face the best thing it showed — the warmest
             * heading, or failing that the glimpse that called the look */
            float target = 0;
            bool have = false;
            if (watch_best_drag > 0) {
                target = watch_best_yaw;
                have = true;
                watch_bedtime_nudge(true);    /* still there when he looked */
            } else if (watch_glimpse_pending) {
                /* the glimpse's angle, mapped into this spin's yaw frame */
                float ysign = watch_yaw >= 0 ? 1.0f : -1.0f;
                target = (watch_glimpse_sign == watch_sign ? ysign : -ysign)
                         * watch_glimpse_deg;
                have = true;
                watch_huh = true;   /* promised, not delivered */
            } else {
                watch_bedtime_nudge(false);   /* an empty circle no glimpse
                                                 even called for */
            }
            watch_glimpse_pending = false;
            if (have) {
                float delta = target - watch_yaw;   /* shortest way back */
                while (delta > 180.0f) {
                    delta -= 360.0f;
                }
                while (delta < -180.0f) {
                    delta += 360.0f;
                }
                if (fabsf(delta) >= WATCH_REORIENT_MIN_DEG) {
                    /* the spin just taught us which drive sign yaws
                     * which way — no calibration constant needed */
                    int d = (delta >= 0) == (watch_yaw >= 0)
                          ? watch_sign : -watch_sign;
                    watch_reorient_deg = fabsf(delta);
                    watch_yaw = 0;
                    watch_look_until =
                        now + WATCH_REORIENT_TIMEOUT_S * 1000000LL;
                    drive(d * base, -d * base);
                    watch_state = WATCH_ORIENT;
                    break;
                }
            }
            watch_look_done(now);
        }
        break;
    }
    case WATCH_ORIENT: {
        watch_yaw += dps[2] * (1.0f / TICK_HZ);
        if (fabsf(watch_yaw) >= watch_reorient_deg || now > watch_look_until) {
            watch_look_done(now);
        }
        break;
    }
    case WATCH_DWELL: {
        /* the double-check is dwell, the cleanest measured fact: sit
         * facing the warmth for 5 s and it must persist — a walk-past
         * spikes hard but never survives this */
        coax_accumulate(t, mean, now);
        if (coax_full) {
            /* right on top of me — staying put */
            watch_settle(now);
            break;
        }
        if (now < coax_until) {
            break;
        }
        float presence = coax_ticks > 0 ? (float)coax_seen / coax_ticks : 0;
        float blob = coax_seen > 0 ? coax_px_sum / coax_seen : 0;
        if (presence < COAX_PRESENCE) {
            watch_huh = true;   /* promised, gone: the head-shake */
            watch_settle(now);
        } else if (blob >= COAX_CEIL_PX) {
            /* already close enough */
            watch_settle(now);
        } else if (coax_lively < COAX_LIVELY) {
            /* the hot window ran hotter than the person; only movement
             * tells them apart — warm furniture gets watched, not met */
            watch_settle(now);
        } else {
            /* a visitor — thinking about it */
            coax_consider(now, presence, blob);
        }
        break;
    }
    case WATCH_NERVE: {
        if (now < coax_until) {
            break;
        }
        float p = coax_p + coax_nerve * COAX_NERVE_BOOST;
        if (p > COAX_P_MAX) {
            p = COAX_P_MAX;
        }
        if (watch_frand() >= p) {
            if (++coax_nerve >= COAX_NERVE_MAX) {
                /* couldn't work up the nerve */
                watch_settle(now);
            } else {
                coax_false_start();   /* almost went */
                coax_until = now + COAX_NERVE_GAP_MS * 1000LL;
            }
            break;
        }
        /* going — face the blob first if it's off by a column or more;
         * drive sign toward an image offset is the field-tested mirror
         * from the glimpse */
        float off = coax_col - WATCH_CENTER_COL;
        coax_turn_deg = fabsf(off) * WATCH_COL_DEG;
        if (coax_turn_deg >= COAX_TURN_MIN_DEG) {
            int d = off > 0 ? -1 : 1;
            watch_yaw = 0;
            coax_until = now + COAX_TURN_TIMEOUT_S * 1000000LL;
            rgb_set(RGB_BLUE);
            drive(d * COAX_TURN_PCT, -d * COAX_TURN_PCT);
            watch_state = WATCH_CTURN;
        } else {
            coax_hop_start(now);
        }
        break;
    }
    case WATCH_CTURN: {
        watch_yaw += dps[2] * (1.0f / TICK_HZ);
        if (fabsf(watch_yaw) >= coax_turn_deg || now > coax_until) {
            coax_hop_start(now);
        }
        break;
    }
    case WATCH_HOP: {
        if (now >= coax_until) {
            drive(0, 0);
            led_nominal();
            coax_window(now, COAX_OBS_SETTLE_MS, COAX_OBS_S);
            watch_state = WATCH_OBS;
        }
        break;
    }
    case WATCH_OBS: {
        coax_accumulate(t, mean, now);
        if (coax_full) {
            /* right on top of me — staying put */
            watch_settle(now);
            break;
        }
        if (now < coax_until) {
            break;
        }
        float presence = coax_ticks > 0 ? (float)coax_seen / coax_ticks : 0;
        float blob = coax_seen > 0 ? coax_px_sum / coax_seen : 0;
        coax_hops++;
        if (presence < COAX_PRESENCE) {
            watch_huh = true;   /* was there, gone: the head-shake */
            watch_settle(now);
        } else if (blob >= COAX_CEIL_PX) {
            /* arrived — a polite metre away, and the visitor becomes
             * wallpaper at the settle's reseed, so waving can't yo-yo him */
            perform_found();
            watch_bedtime_nudge(true);   /* a visit that came off: the
                                            day's best event */
            watch_settle(now);
        } else if (blob < coax_prev_px - COAX_RETREAT_PX) {
            /* backing away — won't chase */
            watch_settle(now);
        } else if (coax_hops >= COAX_HOPS_MAX) {
            /* came this far — your turn */
            watch_settle(now);
        } else {
            coax_consider(now, presence, blob);
        }
        break;
    }
    default:
        break;
    }
}

/* One 10 Hz heartbeat: read every sensor, run hold/gesture detection,
 * act on any gesture, and step the watcher. */
static void tick_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / TICK_HZ));
        float dps[3], g[3], volts = 0, ma = 0;
        int16_t pxbuf[64];
        bool imu_ok = lsm_ok && lsm_read(dps, g) == ESP_OK;
        bool px_ok = amg_ok && amg_read_pixels(pxbuf) == ESP_OK;
        bool pwr_ok = ina_ok && ina_read(&volts, &ma) == ESP_OK;
        if (imu_ok) {
            held_check(dps, g);
        }
        if (gest_toggle) {
            gest_toggle = false;
            watch_toggle();
        }
        if (sprint_go) {
            sprint_go = false;
            perform_sprint();
        }
        if (watch_duty && !held && esp_timer_get_time() >= watch_cycle_at) {
            watch_toggle();   /* waking by himself, or nodding off */
        }
        if (watch_state != WATCH_OFF && imu_ok && px_ok) {
            watch_step(pxbuf, dps, volts, pwr_ok);
        } else if (watch_state == WATCH_OFF && !held &&
                   esp_random() % (DREAM_MED_S * TICK_HZ) == 0) {
            /* dreaming: a rare soft swell of the ember, asleep only */
            rgb_set(RGB_DREAM);
            vTaskDelay(pdMS_TO_TICKS(300));
            led_nominal();
        }
    }
}

void app_main(void)
{
    rgb_init();
    rgb_set(RGB_RED);   /* red until every check passes */

    motors_init();
    i2c_init();
    amg_init();
    ina_init();
    lsm_init();
    xTaskCreate(tick_task, "tick", 4096, NULL, 5, NULL);

    /* The switch-on performance is for a fresh power-on only. A
     * brownout, panic or a glitched EN line (USB DTR/RTS) also lands
     * here, and answering a brownout with a motor dance invites the
     * next one. Failed checks: stay red and still. */
    if (amg_ok && ina_ok && lsm_ok) {
        led_nominal();   /* he boots up asleep: the ember */
        if (esp_reset_reason() == ESP_RST_POWERON) {
            perform_alive();
        }
    }
}
