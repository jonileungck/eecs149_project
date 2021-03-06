// Our master beacon-following robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_clock.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "mpu9250.h"

// Ultrasonic pins
#define US0_PIN BUCKLER_GROVE_A0
#define US1_PIN BUCKLER_GROVE_A1
#define US2_PIN BUCKLER_GROVE_D1

// Calculation parameters
#define pi 3.1415926535897932
#define rad_to_deg 180 / pi
#define speed_of_sound 330
#define disable_time_ms 35
#define offset_update_time_ms 2

// Separations between US0-1, 1-2, 2-0. Need to be measured when testing.
static float US_separations[3] = {.20, .20, .20};

// For printing with display
static char print_str[16];

typedef enum {
  OFF,
  DRIVING,
  TURNING,
  BACK,
  AVOIDL,
  AVOIDR,
  AFORWARD
} robot_state_t;

// LED array
static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);


// ========== Ultrasonic parts starts here ==========
// Ultrasonic detection time keeping
static const nrf_drv_timer_t detection_timer = NRFX_TIMER_INSTANCE(2);

static const nrf_drv_timer_config_t timer_cfg = {
    .frequency          = NRF_TIMER_FREQ_1MHz,
    .mode               = NRF_TIMER_MODE_TIMER,
    .bit_width          = NRF_TIMER_BIT_WIDTH_32,
    .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
    .p_context          = NULL,
};

// Ultrasonic disable timers
APP_TIMER_DEF(disable_timer0);
APP_TIMER_DEF(disable_timer1);
APP_TIMER_DEF(disable_timer2);
APP_TIMER_DEF(offset_update_timer);


static uint32_t disable_ticks = APP_TIMER_TICKS(disable_time_ms);
static uint32_t offset_update_ticks = APP_TIMER_TICKS(offset_update_time_ms);

static uint32_t US_times[3] = {0, 0, 0};
static int time_offset01, time_offset02, time_offset12;

static int counts = 0;

static bool timer_offsets_ready = false;

void calculate_time_offset(void) {
  //__disable_irq();
  time_offset01 = US_times[0] - US_times[1];
  time_offset02 = US_times[0] - US_times[2];
  time_offset12 = US_times[1] - US_times[2];
  ++counts;
  // printf("%i: US0: %lu, US1: %lu, US2: %lu\n", counts, US_times[0], US_times[1], US_times[2]);
  // printf("%i: 01: %i, 02: %i, 12: %i\n", counts, time_offset01, time_offset02, time_offset12);
  //__enable_irq();
}

void detection_timer_event_handler(nrf_timer_event_t event_type, void *p_context) {
  // don't care
}

void US0_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  US_times[0] = nrfx_timer_capture(&detection_timer, NRF_TIMER_CC_CHANNEL0);
  nrfx_gpiote_in_event_disable(US0_PIN);
  nrfx_gpiote_out_clear(LEDS[2]);
  timer_offsets_ready = false;

  ret_code_t error_code = app_timer_start(disable_timer0, disable_ticks, NULL);
  APP_ERROR_CHECK(error_code);

  error_code = app_timer_start(offset_update_timer, offset_update_ticks, NULL);
  APP_ERROR_CHECK(error_code);
}

void US1_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  US_times[1] = nrfx_timer_capture(&detection_timer, NRF_TIMER_CC_CHANNEL1);
  nrfx_gpiote_in_event_disable(US1_PIN);
  nrfx_gpiote_out_clear(LEDS[0]);
  timer_offsets_ready = false;

  ret_code_t error_code = app_timer_start(disable_timer1, disable_ticks, NULL);
  APP_ERROR_CHECK(error_code);

  error_code = app_timer_start(offset_update_timer, offset_update_ticks, NULL);
  APP_ERROR_CHECK(error_code);
}

void US2_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  US_times[2] = nrfx_timer_capture(&detection_timer, NRF_TIMER_CC_CHANNEL2);
  nrfx_gpiote_in_event_disable(US2_PIN);
  nrfx_gpiote_out_clear(LEDS[1]);
  timer_offsets_ready = false;

  ret_code_t error_code = app_timer_start(disable_timer2, disable_ticks, NULL);
  APP_ERROR_CHECK(error_code);

  error_code = app_timer_start(offset_update_timer, offset_update_ticks, NULL);
  APP_ERROR_CHECK(error_code);
}

void offset_update_timer_event_handler(void* p_context) {
  calculate_time_offset();
  timer_offsets_ready = true;
}

void disable_timer0_event_handler(void* p_context) {
  // turn on interrupt
  nrfx_gpiote_in_event_enable(US0_PIN, true);
  nrfx_gpiote_out_set(LEDS[2]);
}

void disable_timer1_event_handler(void* p_context) {
  // turn on interrupt
  nrfx_gpiote_in_event_enable(US1_PIN, true);
  nrfx_gpiote_out_set(LEDS[0]);
}

void disable_timer2_event_handler(void* p_context) {
  // turn on interrupt
  nrfx_gpiote_in_event_enable(US2_PIN, true);
  nrfx_gpiote_out_set(LEDS[1]);
}

// Function starting the internal LFCLK oscillator.
// This is needed by RTC1 which is used by the application timer
// (When SoftDevice is enabled the LFCLK is always running and this is not needed).
static void lfclk_request(void)
{
    ret_code_t error_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(error_code);
    nrf_drv_clock_lfclk_request(NULL);
}

// Create all app timers needed
static void create_app_timers(void) {
  ret_code_t error_code = NRF_SUCCESS;
  error_code = app_timer_create(&disable_timer0,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                disable_timer0_event_handler);
  APP_ERROR_CHECK(error_code);

  error_code = app_timer_create(&disable_timer1,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                disable_timer1_event_handler);
  APP_ERROR_CHECK(error_code);

  error_code = app_timer_create(&disable_timer2,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                disable_timer2_event_handler);
  APP_ERROR_CHECK(error_code);

  error_code = app_timer_create(&offset_update_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                offset_update_timer_event_handler);
  APP_ERROR_CHECK(error_code);
}

float calculate_target_angle(void) {
  //__disable_irq();
  float angle, ratio;
  ratio = -time_offset01 * speed_of_sound / US_separations[0] / 1000000;
  // When ready to deploy, replace if cases with these two lines.
  ratio = ratio > 1 ? 1 : ratio;
  ratio = ratio < -1 ? -1 : ratio;
  // printf("Ratio: %f\n", ratio);
  angle = acos(ratio) * rad_to_deg;
  // printf("raw angle: %f\n", angle);
  snprintf(print_str, 16, "%f", angle);
  display_write(print_str, DISPLAY_LINE_0);
  angle = angle - 90;
  if (time_offset02 < -US_separations[1] * speed_of_sound / 2 && time_offset12 < 0) {
    angle -= 90;
  } else if (time_offset02 < 0 && time_offset12 < -US_separations[2] * speed_of_sound / 2) {
    angle += 90;
  }

  if (time_offset02 < 0 && time_offset12 < 0) {
    if (angle > 0) {
      angle += 90;
    } else {
      angle -= 90;
    }
  }
  snprintf(print_str, 16, "%f", angle);
  display_write(print_str, DISPLAY_LINE_1);
  //__enable_irq();
  return angle/2;
}

// ========== Ultrasonic parts ends here ==========

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
    const float CONVERSION = 0.00008529;
    float result_edge_case = 0;

    if (current_encoder < previous_encoder && previous_encoder - current_encoder > 0x7FFF) {
        return (0xFFFF - previous_encoder + current_encoder)*CONVERSION;
    } 
    return (current_encoder - previous_encoder)*CONVERSION;
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  // Initialize timer
  error_code = nrfx_timer_init(&detection_timer, &timer_cfg, detection_timer_event_handler);
  APP_ERROR_CHECK(error_code);
  nrfx_timer_clear(&detection_timer);
  nrfx_timer_enable(&detection_timer);

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize GPIO driver
  if (!nrfx_gpiote_is_init()) {
    error_code = nrfx_gpiote_init();
  }

  // Initialize LEDs for debugging
  nrfx_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
  for (int i=0; i<3; i++) {
    error_code = nrfx_gpiote_out_init(LEDS[i], &out_config);
    APP_ERROR_CHECK(error_code);
    nrfx_gpiote_out_set(LEDS[i]);
  }

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  mpu9250_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // Initialize ultrasonic interrupts
  nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  in_config.pull = NRF_GPIO_PIN_PULLDOWN;

  error_code = nrfx_gpiote_in_init(US0_PIN, &in_config, US0_handler);
  nrfx_gpiote_in_event_enable(US0_PIN, true);
  APP_ERROR_CHECK(error_code);

  error_code = nrfx_gpiote_in_init(US1_PIN, &in_config, US1_handler);
  nrfx_gpiote_in_event_enable(US1_PIN, true);
  APP_ERROR_CHECK(error_code);

  error_code = nrfx_gpiote_in_init(US2_PIN, &in_config, US2_handler);
  nrfx_gpiote_in_event_enable(US2_PIN, true);
  APP_ERROR_CHECK(error_code);

  // Initialize app timer
  lfclk_request();
  error_code = app_timer_init();
  APP_ERROR_CHECK(error_code);
  create_app_timers();

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

  float target_angle, current_angle, distance;

  bool bump_left, bump_right, avoid_left, avoid_right, direction;
  direction = true; // forward;
  uint16_t right_whl_encoder_curr, right_whl_encoder_prev = 0;

  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);
    bump_left = sensors.bumps_wheelDrops.bumpLeft;
    bump_right = sensors.bumps_wheelDrops.bumpRight | sensors.bumps_wheelDrops.bumpCenter;
    right_whl_encoder_curr = sensors.rightWheelEncoder;
    distance = measure_distance(right_whl_encoder_curr, right_whl_encoder_prev);
    printf("%f, %i, %i\n", distance, right_whl_encoder_curr, right_whl_encoder_prev);

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(100);

    // handle states
    switch(state) {
      case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = DRIVING;
        } else {
          // perform state-specific actions here
          display_write("OFF", DISPLAY_LINE_0);
          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVING: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (bump_left) {
            right_whl_encoder_prev = right_whl_encoder_curr;
            direction = false;
            target_angle = -45;
            current_angle = 0;
            mpu9250_start_gyro_integration();
            state = AVOIDL;
        } else if (bump_right) {
            right_whl_encoder_prev = right_whl_encoder_curr;
            direction = false;
            target_angle = 45;
            current_angle = 0;
            mpu9250_start_gyro_integration();
            state = AVOIDR;
        } else if (timer_offsets_ready) {
          timer_offsets_ready = false;
          target_angle = calculate_target_angle();
          snprintf(print_str, 16, "%f", target_angle);
          display_write(print_str, DISPLAY_LINE_1);
          if (target_angle > 10 || target_angle < -10) {
            current_angle = 0;
            mpu9250_start_gyro_integration();
            state = TURNING;
          } else {
            display_write("AHEAD", DISPLAY_LINE_1);
            state = DRIVING;
          }
        } else {
          // perform state-specific actions here
          display_write("DRIVING", DISPLAY_LINE_0);
          snprintf(print_str, 16, "%f", distance);
          display_write(print_str, DISPLAY_LINE_1);
          kobukiDriveDirect(100, 100);
          state = DRIVING;
        }
        break; // each case needs to end with break!
      }

      case TURNING: {
        if (is_button_pressed(&sensors)) {
          target_angle = 0;
          current_angle = 0;
          mpu9250_stop_gyro_integration();
          state = OFF;
        } else {
          //display_write("TURNING", DISPLAY_LINE_0);
          current_angle = mpu9250_read_gyro_integration().z_axis;
          snprintf(print_str, 16, "%f", target_angle);
          display_write(print_str, DISPLAY_LINE_0);
          snprintf(print_str, 16, "%f", current_angle);
          display_write(print_str, DISPLAY_LINE_1);
          if (target_angle > 0 && target_angle - current_angle > 5) {
            kobukiDriveDirect(-100, 100);
            state = TURNING;
          } else if (target_angle < 0 && target_angle - current_angle < -5) {
            kobukiDriveDirect(100, -100);
            state = TURNING;
          } else {
            mpu9250_stop_gyro_integration();
            target_angle = 0;
            current_angle = 0;
            state = DRIVING;
          }
        }
        break;
      }

      case AVOIDL: {
        display_write("AVOIDL", DISPLAY_LINE_0);
        snprintf(print_str, 16, "%f", distance);
        display_write(print_str, DISPLAY_LINE_1);
        if (is_button_pressed(&sensors)) {
          target_angle = 0;
          current_angle = 0;
          right_whl_encoder_prev = right_whl_encoder_curr;
          direction = true;
          mpu9250_stop_gyro_integration();
          state = OFF;
        } else {
            if (bump_left) {
                right_whl_encoder_prev = right_whl_encoder_curr;
                kobukiDriveDirect(-100, -100);
                direction = false;
                target_angle = -45;
                current_angle = 0;
                mpu9250_stop_gyro_integration();
                mpu9250_start_gyro_integration();
                state = AVOIDL;
            } else if (bump_right) {
                right_whl_encoder_prev = right_whl_encoder_curr;
                kobukiDriveDirect(-100, -100);
                direction = false;
                target_angle = 45;
                current_angle = 0;
                mpu9250_stop_gyro_integration();
                mpu9250_start_gyro_integration();
                state = AVOIDR;
            } else if (distance > -0.1 && !direction) {
                kobukiDriveDirect(-100, -100);
            } else if (current_angle > target_angle) {
                current_angle = mpu9250_read_gyro_integration().z_axis;
                direction = true;
                right_whl_encoder_prev = right_whl_encoder_curr;
                right_whl_encoder_prev = right_whl_encoder_curr;
                kobukiDriveDirect(100, -100);
            } else if (distance < 0.2) {
                kobukiDriveDirect(100, 100);
            } else {
                target_angle = 0;
                current_angle = 0;
                right_whl_encoder_prev = right_whl_encoder_curr;
                mpu9250_stop_gyro_integration();
                state = DRIVING;
            }
        }
        break;
      }

      case AVOIDR: {
        display_write("AVOIDR", DISPLAY_LINE_0);
        snprintf(print_str, 16, "%f", distance);
        display_write(print_str, DISPLAY_LINE_1);
        if (is_button_pressed(&sensors)) {
          target_angle = 0;
          current_angle = 0;
          right_whl_encoder_prev = right_whl_encoder_curr;
          direction = true;
          mpu9250_stop_gyro_integration();
          state = OFF;
        } else {
            if (bump_left) {
                right_whl_encoder_prev = right_whl_encoder_curr;
                kobukiDriveDirect(-100, -100);
                direction = false;
                target_angle = -45;
                current_angle = 0;
                mpu9250_stop_gyro_integration();
                mpu9250_start_gyro_integration();
                state = AVOIDL;
            } else if (bump_right) {
                right_whl_encoder_prev = right_whl_encoder_curr;
                kobukiDriveDirect(-100, -100);
                direction = false;
                target_angle = 45;
                current_angle = 0;
                mpu9250_stop_gyro_integration();
                mpu9250_start_gyro_integration();
                state = AVOIDR;
            } else if (distance > -0.1 && !direction) {
                kobukiDriveDirect(-100, -100);
            } else if (current_angle < target_angle) {
                current_angle = mpu9250_read_gyro_integration().z_axis;
                direction = true;
                right_whl_encoder_prev = right_whl_encoder_curr;
                kobukiDriveDirect(-100, 100);
            } else if (distance < 0.2) {
                kobukiDriveDirect(100, 100);
            } else {
                target_angle = 0;
                current_angle = 0;
                right_whl_encoder_prev = right_whl_encoder_curr;
                mpu9250_stop_gyro_integration();
                state = DRIVING;
            }
        }
        break;
      }


      // add other cases here

    }
  }
}

