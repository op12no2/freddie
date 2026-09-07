/* firmware: Freddie the robot. AMG8833 thermal camera, LSM6DSOX IMU and
 * INA219 power monitor on one I2C bus, DRV8833 motors, and the DevKit's
 * onboard RGB status LED.
 *
 * The whole behaviour: spin one circle on the spot, slowing as warmth
 * crosses the view, then rest half a minute and spin again; when something
 * warm sits near the middle of the frame, drive at it, steering on its
 * centroid, until it fills the frame; sit there while it does; lose it
 * and go back to the circling. Nothing else. */

#include <math.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
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
 * Red = booting or a check failed, green = scanning (an ember of it
 * between scans), blue = going for something, amber = arrived. */
#define RGB_RED         32, 0, 0
#define RGB_GREEN       0, 32, 0
#define RGB_GREEN_DIM   0, 5, 0
#define RGB_BLUE        0, 0, 48
#define RGB_AMBER       48, 24, 0

#define WAKE_LED_GPIO   11     /* discrete orange LED, 1 kOhm to GND, on the
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

    gpio_config_t wake_cfg = {
        .pin_bit_mask = 1ULL << WAKE_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&wake_cfg));
    gpio_set_level(WAKE_LED_GPIO, 0);   /* dark until the checks pass */
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

/* True when the pack is supplying current. With the pack off and USB in,
 * the 6V rail is back-fed through the DevKit's diode and the pack shunt
 * carries nothing — running motors then would pull amps through that
 * diode (see schematic). The pack covers the ESP32's idle draw
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
 * wheels are driven. Set by open-loop test. Measured: actual duties
 * 49/50 drive straight. */
#define DRIVE_TRIM_PCT  -1

static void drive(int left_pct, int right_pct)
{
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
#define SPIN_PCT       20     /* scan duty (pre-remap) */
#define SCAN_TURN_DEG  360.0f /* one gyro-metered circle per scan */
#define SCAN_TIMEOUT_S 30     /* gyro trouble: don't pirouette forever */
#define REST_S         30     /* still, between scans */
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
#define GO_PCT         30     /* approach duty */
#define STEER_K        6.0f   /* duty differential per column off boresight */
#define FILL_PX        32     /* half the frame = arrived. 1 m from a
                                 crouching child fills 14 px (gestures.log);
                                 half the frame is right up close */
#define FILL_HYST_PX   8      /* it must shrink this much below FILL_PX
                                 before he follows again */
#define LOST_TICKS     10     /* a second without the target = gone */

typedef enum { SCAN, REST, APPROACH, ARRIVED } state_t;

static state_t state;
static int scan_sign;      /* spin direction this scan */
static float scan_yaw;     /* degrees turned this scan */
static int ticks_left;     /* scan timeout, or rest remaining */
static float ambient;      /* frozen reference the target is measured
                              against while approaching — the frame mean
                              can't serve, a target that fills the frame
                              *is* the mean */
static int lost;           /* consecutive ticks without the target */

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

/* Mean of the pixels that aren't the blob: the scene minus the visitor. */
static float ambient_of(const float t[64], float mean)
{
    int n = 0;
    float sum = 0;
    for (int i = 0; i < 64; i++) {
        if (t[i] - mean < BLOB_C) {
            n++;
            sum += t[i];
        }
    }
    return n > 0 ? sum / n : mean;
}

static void enter(state_t s)
{
    state = s;
    lost = 0;
    switch (s) {
    case SCAN:
        scan_sign = (esp_random() & 1) ? 1 : -1;
        scan_yaw = 0;
        ticks_left = SCAN_TIMEOUT_S * TICK_HZ;
        rgb_set(RGB_GREEN);
        break;
    case REST:
        drive(0, 0);
        ticks_left = REST_S * TICK_HZ;
        rgb_set(RGB_GREEN_DIM);
        break;
    case APPROACH:
        rgb_set(RGB_BLUE);
        break;
    case ARRIVED:
        drive(0, 0);
        rgb_set(RGB_AMBER);
        break;
    }
}

static int clamp_pct(int pct)
{
    return pct < 0 ? 0 : pct > 100 ? 100 : pct;
}

/* A warm blob sitting near boresight: the thing he goes for. */
static bool locked(const float t[64], float mean, float *col)
{
    return blob(t, mean, col) > 0 && fabsf(*col - CENTER_COL) <= LOCK_COLS;
}

/* One 10 Hz heartbeat: read the camera and gyro, step the behaviour. */
static void tick_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    enter(SCAN);
    while (1) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / TICK_HZ));
        int16_t px[64];
        float dps[3] = { 0 }, g[3];
        if (amg_read_pixels(px) != ESP_OK) {
            drive(0, 0);   /* blind: don't move */
            continue;
        }
        lsm_read(dps, g);   /* a failed read counts no yaw; the scan
                               timeout covers it */
        float t[64], maxt = -100, sum = 0, col = CENTER_COL;
        for (int i = 0; i < 64; i++) {
            t[i] = px[i] * 0.25f;
            sum += t[i];
            if (t[i] > maxt) {
                maxt = t[i];
            }
        }
        float mean = sum / 64;

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
            if (locked(t, mean, &col)) {
                ambient = ambient_of(t, mean);
                enter(APPROACH);
                break;
            }
            scan_yaw += dps[2] * (1.0f / TICK_HZ);
            if (fabsf(scan_yaw) >= SCAN_TURN_DEG || --ticks_left <= 0) {
                enter(REST);   /* circle done, nothing doing */
            }
            break;
        }
        case REST: {
            /* still, but not blind: something warm walking up to him
             * mid-rest gets the same welcome as it would mid-scan */
            if (locked(t, mean, &col)) {
                ambient = ambient_of(t, mean);
                enter(APPROACH);
                break;
            }
            if (--ticks_left <= 0) {
                enter(SCAN);
            }
            break;
        }
        case APPROACH: {
            int n = blob(t, ambient, &col);
            if (n == 0) {
                if (++lost >= LOST_TICKS) {
                    enter(SCAN);
                }
                break;   /* a dropped frame: hold course */
            }
            lost = 0;
            if (n >= FILL_PX) {
                enter(ARRIVED);
                break;
            }
            /* sign field-tested: image columns run mirrored, so a
             * centroid right of boresight means the target is to his
             * left — slow the left wheel */
            float off = col - CENTER_COL;
            drive(clamp_pct(GO_PCT - (int)(STEER_K * off)),
                  clamp_pct(GO_PCT + (int)(STEER_K * off)));
            break;
        }
        case ARRIVED: {
            int n = blob(t, ambient, &col);
            if (n == 0) {
                if (++lost >= LOST_TICKS) {
                    enter(SCAN);
                }
                break;
            }
            lost = 0;
            if (n < FILL_PX - FILL_HYST_PX) {
                enter(APPROACH);   /* they stepped back: follow */
            }
            break;
        }
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
    if (amg_ok && ina_ok && lsm_ok) {
        gpio_set_level(WAKE_LED_GPIO, 1);
        xTaskCreate(tick_task, "tick", 4096, NULL, 5, NULL);
    }
}
