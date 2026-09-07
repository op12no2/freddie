/* firmware: Freddie the robot. AMG8833 thermal camera, LSM6DSOX IMU and
 * INA219 power monitor on one I2C bus, DRV8833 motors, and the DevKit's
 * onboard RGB status LED, and a discrete "running" LED on the shelf.
 *
 * The whole behaviour: spin on the spot, slowing as warmth crosses the
 * view; when something warm sits near the middle of the frame, chase it
 * flat out, steering on its centroid, until he loses it or runs into it;
 * either way, look around again (a run-in earns a back-off and a turn
 * away first). He's the one doing the tagging; it's on you to get out of
 * the way. Nothing else.
 *
 * A serial console (idf.py monitor, '?' for help) shows the thermal
 * frame for tuning; it's for the bench, not the behaviour. */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "driver/uart.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
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
 * Red = booting or a check failed, green = scanning, blue = following,
 * violet = tagged. */
#define RGB_RED         32, 0, 0
#define RGB_GREEN       0, 32, 0
#define RGB_BLUE        0, 0, 48
#define RGB_TAG         48, 0, 48

#define RUN_LED_GPIO    11     /* discrete orange LED, 1 kOhm to GND, on the
                                  shelf: lit whenever he's running */

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

    gpio_config_t run_cfg = {
        .pin_bit_mask = 1ULL << RUN_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&run_cfg));
    gpio_set_level(RUN_LED_GPIO, 0);   /* dark until the checks pass */
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

/* The pack, as last read by the tick. */
static float pack_v, pack_ma;
static bool pack_ok;

/* True when the pack is supplying current. With the pack off and USB in,
 * the 6V rail is back-fed through the DevKit's diode and the pack shunt
 * carries nothing — running motors then would pull amps through that
 * diode (see schematic). The pack covers the ESP32's idle draw
 * whenever it's on (measured 43-59 mA untethered, ~0 on USB), so a
 * near-zero shunt reading means USB only. */
static bool pack_live(void)
{
    if (!pack_ok) {
        return true;   /* can't tell — assume the pack is on */
    }
    return pack_ma > 20.0f;
}

/* No two TT motors are matched: the trim is added to the left duty
 * (sign-aware, so it corrects magnitude in reverse too) whenever both
 * wheels are driven. Set by open-loop test. Measured: actual duties
 * 49/50 drive straight. */
#define DRIVE_TRIM_PCT  -1

static int cmd_left, cmd_right;   /* commanded duties, so the stall check
                                     knows when he's meant to be moving */

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
    /* The DRV8833 sleeps whenever he's still. Wake time is well under a
     * PWM period. */
    gpio_set_level(DRV_SLP_GPIO, left_pct != 0 || right_pct != 0);
    motor_set(MOTOR_L_IN1_CH, MOTOR_L_IN2_CH, left_pct);
    motor_set(MOTOR_R_IN1_CH, MOTOR_R_IN2_CH, right_pct);
}

/* Behaviour knobs. Thresholds cite the logs they were measured from
 * (gestures.log, quiet_room_sat.log), recorded before the firmware was
 * simplified; the numbers are load-bearing even though the recording
 * machinery is gone. */
#define TICK_HZ        10     /* the sensor/behaviour heartbeat */
#define SETTLE_S       2      /* still after boot while the camera's first
                                 frames, which are junk, go by */
#define LOCK_TICKS     3      /* the target must sit near boresight this
                                 many ticks running before he goes: one
                                 noisy frame can't launch him */
#define SPIN_PCT       30     /* scan duty (pre-remap) */
#define SPIN_MIN_PCT   1      /* gaze-drag floor: linger, never stall */
#define GAZE_K         8.0f   /* duty shed per C of passing warmth */
#define GAZE_DEAD_C    0.8f   /* scene contrast to ignore (empty-room
                                 max-mean runs ~0.9-1.4, gestures.log) */
#define BLOB_C         2.0f   /* px over ambient = part of the target: a
                                 standing person at 2 m clears the frame
                                 mean by +2.0-2.4 C (gestures.log) */
#define BLOB_MIN_PX    2      /* one noise pixel (±2.5 C) isn't a target */
#define CENTER_COL     3.2f   /* boresight column (measured) */
#define LOCK_COLS      1.0f   /* blob this near boresight during the scan:
                                 go for it */
#define GO_PCT         100    /* follow duty: a chase, all the way in. No
                                 slowing, no stopping short — he's small,
                                 and getting out of his way is the game */
#define STEER_K        25.0f  /* inner-wheel duty shed per column the
                                 target sits off boresight */
#define LOST_TICKS     10     /* stand still this long without the target,
                                 then look around */
#define STALL_MA       900.0f /* pack current with the motors commanded =
                                 pushing on something (feet, wall). A GUESS:
                                 measure with the console, 's' while he
                                 chases and again while you hold him back,
                                 and put the number between them here */
#define STALL_TICKS    3      /* sustained: launch inrush is a tick or so */
#define BACK_PCT       60     /* tagged: reverse... */
#define BACK_MS        400
#define TURN_PCT       60     /* ...and about-face, gyro-metered, to a
                                 random heading in this range */
#define TURN_MIN_DEG   120
#define TURN_MAX_DEG   240
#define TURN_TIMEOUT_S 3      /* gyro trouble: don't pirouette forever */

typedef enum { SCAN, FOLLOW, BACK, TURN } state_t;

static state_t state;
static int scan_sign;      /* spin direction this scan */
static int lost;           /* consecutive ticks without the target */
static int locking;        /* consecutive ticks with a target near boresight */
static int stalled;        /* consecutive ticks pushing on something */
static int ticks_left;     /* back-off remaining, or turn timeout */
static float turn_deg;     /* the about-face: degrees wanted... */
static float turn_yaw;     /* ...and turned so far */
static int turn_sign;

/* For the console: the latest frame. */
static float last_t[64];
static volatile bool frozen;   /* console 'x': sense, but don't move */

static int cmp_float(const void *a, const void *b)
{
    float d = *(const float *)a - *(const float *)b;
    return d < 0 ? -1 : d > 0 ? 1 : 0;
}

/* The scene minus whoever's in it: the mean of the coldest half of the
 * frame. Holds up until a target covers half the view, at which point he
 * can't tell them from the wall anyway — and that's when he loses them
 * and looks around, which is how the chase ends. Beats the frame mean (a
 * target that fills the frame *is* the mean) and a reference frozen at
 * lock-on (stale by the time he's crossed the room). */
static float ambient_of(const float t[64])
{
    float sorted[64];
    for (int i = 0; i < 64; i++) {
        sorted[i] = t[i];
    }
    qsort(sorted, 64, sizeof(float), cmp_float);
    float sum = 0;
    for (int i = 0; i < 32; i++) {
        sum += sorted[i];
    }
    return sum / 32;
}

/* Blob of pixels BLOB_C over ref: its weight in pixels, and its centroid
 * column when there is one. */
static int blob(const float t[64], float ref, float *col)
{
    int n = 0;
    float csum = 0;
    for (int i = 0; i < 64; i++) {
        if (t[i] - ref >= BLOB_C) {
            n++;
            csum += i % 8;
        }
    }
    if (n < BLOB_MIN_PX) {
        return 0;
    }
    *col = csum / n;
    return n;
}

static void enter(state_t s)
{
    state = s;
    lost = 0;
    locking = 0;
    stalled = 0;
    switch (s) {
    case SCAN:
        scan_sign = (esp_random() & 1) ? 1 : -1;
        rgb_set(RGB_GREEN);
        break;
    case FOLLOW:
        rgb_set(RGB_BLUE);
        break;
    case BACK:
        ticks_left = BACK_MS * TICK_HZ / 1000;
        drive(-BACK_PCT, -BACK_PCT);
        rgb_set(RGB_TAG);
        break;
    case TURN:
        turn_deg = TURN_MIN_DEG + esp_random() % (TURN_MAX_DEG - TURN_MIN_DEG + 1);
        turn_yaw = 0;
        turn_sign = (esp_random() & 1) ? 1 : -1;
        ticks_left = TURN_TIMEOUT_S * TICK_HZ;
        drive(turn_sign * TURN_PCT, -turn_sign * TURN_PCT);
        break;
    }
}

static int clamp_pct(int pct)
{
    return pct < 0 ? 0 : pct > 100 ? 100 : pct;
}

/* One 10 Hz heartbeat: read the camera, the pack and the gyro, step the
 * behaviour. */
static void tick_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(SETTLE_S * 1000));
    TickType_t wake = xTaskGetTickCount();
    enter(SCAN);
    while (1) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / TICK_HZ));
        int16_t px[64];
        float dps[3] = { 0 }, g[3];
        pack_ok = ina_ok && ina_read(&pack_v, &pack_ma) == ESP_OK;
        lsm_read(dps, g);   /* a failed read counts no yaw; the turn
                               timeout covers it */
        if (amg_read_pixels(px) != ESP_OK) {
            drive(0, 0);   /* blind: don't move */
            continue;
        }
        float t[64], maxt = -100, sum = 0, col = CENTER_COL;
        for (int i = 0; i < 64; i++) {
            t[i] = px[i] * 0.25f;
            last_t[i] = t[i];
            sum += t[i];
            if (t[i] > maxt) {
                maxt = t[i];
            }
        }
        float mean = sum / 64;
        int n = blob(t, ambient_of(t), &col);
        if (frozen) {
            drive(0, 0);
            continue;
        }

        switch (state) {
        case SCAN: {
            /* the gaze lingers: passing warmth sheds spin duty */
            float drag = (maxt - mean) - GAZE_DEAD_C;
            if (drag < 0) {
                drag = 0;
            }
            int duty = SPIN_PCT - (int)(GAZE_K * drag);
            if (duty < SPIN_MIN_PCT) {
                duty = SPIN_MIN_PCT;
            }
            drive(scan_sign * duty, -scan_sign * duty);
            bool near = n > 0 && fabsf(col - CENTER_COL) <= LOCK_COLS;
            locking = near ? locking + 1 : 0;
            if (locking >= LOCK_TICKS) {
                enter(FOLLOW);
            }
            break;
        }
        case FOLLOW: {
            /* tag: commanded moving but the pack says he's pushing on
             * something — feet, usually */
            bool pushing = pack_ok && (cmd_left != 0 || cmd_right != 0) &&
                           pack_ma > STALL_MA;
            stalled = pushing ? stalled + 1 : 0;
            if (stalled >= STALL_TICKS) {
                enter(BACK);
                break;
            }
            if (n == 0) {
                drive(0, 0);   /* don't charge blind at full duty */
                if (++lost >= LOST_TICKS) {
                    enter(SCAN);
                }
                break;
            }
            lost = 0;
            /* sign field-tested: image columns run mirrored, so a
             * centroid right of boresight means the target is to his
             * left — shed the left wheel */
            int turn = (int)(STEER_K * (col - CENTER_COL));
            drive(clamp_pct(GO_PCT - (turn > 0 ? turn : 0)),
                  clamp_pct(GO_PCT + (turn < 0 ? turn : 0)));
            break;
        }
        case BACK: {
            if (--ticks_left <= 0) {
                enter(TURN);
            }
            break;
        }
        case TURN: {
            turn_yaw += dps[2] * (1.0f / TICK_HZ);
            if (fabsf(turn_yaw) >= turn_deg || --ticks_left <= 0) {
                drive(0, 0);
                enter(SCAN);
            }
            break;
        }
        }
    }
}

static const char *state_name(void)
{
    switch (state) {
    case SCAN: return "scan";
    case FOLLOW: return "follow";
    case BACK: return "back";
    case TURN: return "turn";
    }
    return "?";
}

/* The frame as the sensor delivers it: row 0 first, column 0 first.
 * Pixels the behaviour counts as the target are starred. */
static void print_frame(void)
{
    float t[64];
    for (int i = 0; i < 64; i++) {
        t[i] = last_t[i];
    }
    float ref = ambient_of(t);
    float col = 0;
    int n = blob(t, ref, &col);
    printf("      ");
    for (int c = 0; c < 8; c++) {
        printf("   c%d ", c);
    }
    printf("\n");
    for (int r = 0; r < 8; r++) {
        printf("  r%d  ", r);
        for (int c = 0; c < 8; c++) {
            float v = t[r * 8 + c];
            printf("%5.1f%c", v, v - ref >= BLOB_C ? '*' : ' ');
        }
        printf("\n");
    }
    printf("  %s%s  ambient %.2f  blob %d px", state_name(),
           frozen ? " (frozen)" : "", ref, n);
    if (n > 0) {
        printf("  col %.2f (off %+.2f)", col, col - CENTER_COL);
    }
    printf("\n  pack %.2f V %.0f mA  motors %d/%d\n", pack_v, pack_ma,
           cmd_left, cmd_right);
}

static void print_help(void)
{
    printf("freddie console: ? help, p frame, s stream frames (any key stops), "
           "x freeze/unfreeze motors\n");
}

static void console(void)
{
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    print_help();
    bool stream = false;
    while (1) {
        uint8_t c;
        if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(500)) <= 0) {
            if (stream) {
                print_frame();
            }
            continue;
        }
        if (stream) {
            stream = false;   /* any key stops the stream */
            continue;
        }
        switch (c) {
        case '?':
            print_help();
            break;
        case 'p':
            print_frame();
            break;
        case 's':
            stream = true;
            break;
        case 'x':
            frozen = !frozen;
            printf("motors %s\n", frozen ? "frozen" : "free");
            break;
        default:
            break;
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

    /* Failed checks: stay red and still. */
    printf("startup: amg %s, ina %s, lsm %s\n", amg_ok ? "ok" : "MISSING",
           ina_ok ? "ok" : "MISSING", lsm_ok ? "ok" : "MISSING");
    if (amg_ok && ina_ok && lsm_ok) {
        gpio_set_level(RUN_LED_GPIO, 1);
        rgb_set(RGB_GREEN);   /* checks passed; still while the camera settles */
        xTaskCreate(tick_task, "tick", 4096, NULL, 5, NULL);
    }
    console();   /* never returns */
}
