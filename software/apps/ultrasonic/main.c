#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrfx_gpiote.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_clock.h"

#include "buckler.h"

// Ultrasonic pins
#define US0_PIN BUCKLER_GROVE_A0
#define US1_PIN BUCKLER_GROVE_A1
#define US2_PIN BUCKLER_GROVE_D1

// Calculation parameters
#define pi 3.1415926535897932
#define rad_to_deg 180 / pi
#define speed_of_sound 330

// Separations between US0-1, 1-2, 2-0. Need to be measured when testing.
static float US_separations[3] = {10, 10, 10};

// LED array
static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};

// Ultrasonic detection time keeping
static const nrf_drv_timer_t detection_timer = NRFX_TIMER_INSTANCE(1);

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

static uint32_t disable_ticks = APP_TIMER_TICKS(35);
static uint32_t offset_update_ticks = APP_TIMER_TICKS(2);

static uint32_t US_times[3] = {0, 0, 0};
static int time_offset01, time_offset02, time_offset12;

static int counts = 0;

void calculate_time_offset(void) {
  __disable_irq();
  time_offset01 = US_times[0] - US_times[1];
  time_offset02 = US_times[0] - US_times[2];
  time_offset12 = US_times[1] - US_times[2];
  ++counts;
  //printf("%i: US0: %lu, US1: %lu, US2: %lu\n", counts, US_times[0], US_times[1], US_times[2]);
  printf("%i: 01: %i, 02: %i, 12: %i\n", counts, time_offset01, time_offset02, time_offset12);
  __enable_irq();
}

void detection_timer_event_handler(nrf_timer_event_t event_type, void *p_context) {
  // don't care
}

void US0_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  US_times[0] = nrfx_timer_capture(&detection_timer, NRF_TIMER_CC_CHANNEL0);
  nrfx_gpiote_in_event_disable(US0_PIN);
  nrfx_gpiote_out_clear(LEDS[0]);

  ret_code_t error_code = app_timer_start(disable_timer0, disable_ticks, NULL);
  APP_ERROR_CHECK(error_code);

  error_code = app_timer_start(offset_update_timer, offset_update_ticks, NULL);
  APP_ERROR_CHECK(error_code);
}

void US1_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  US_times[1] = nrfx_timer_capture(&detection_timer, NRF_TIMER_CC_CHANNEL1);
  nrfx_gpiote_in_event_disable(US1_PIN);
  nrfx_gpiote_out_clear(LEDS[1]);

  ret_code_t error_code = app_timer_start(disable_timer1, disable_ticks, NULL);
  APP_ERROR_CHECK(error_code);

  error_code = app_timer_start(offset_update_timer, offset_update_ticks, NULL);
  APP_ERROR_CHECK(error_code);
}

void US2_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  US_times[2] = nrfx_timer_capture(&detection_timer, NRF_TIMER_CC_CHANNEL2);
  nrfx_gpiote_in_event_disable(US2_PIN);
  nrfx_gpiote_out_clear(LEDS[2]);

  ret_code_t error_code = app_timer_start(disable_timer2, disable_ticks, NULL);
  APP_ERROR_CHECK(error_code);

  error_code = app_timer_start(offset_update_timer, offset_update_ticks, NULL);
  APP_ERROR_CHECK(error_code);
}

void offset_update_timer_event_handler(void* p_context) {
  calculate_time_offset();
}

void disable_timer0_event_handler(void* p_context) {
  // turn on interrupt
  nrfx_gpiote_in_event_enable(US0_PIN, true);
  nrfx_gpiote_out_set(LEDS[0]);
}

void disable_timer1_event_handler(void* p_context) {
  // turn on interrupt
  nrfx_gpiote_in_event_enable(US1_PIN, true);
  nrfx_gpiote_out_set(LEDS[1]);
}

void disable_timer2_event_handler(void* p_context) {
  // turn on interrupt
  nrfx_gpiote_in_event_enable(US2_PIN, true);
  nrfx_gpiote_out_set(LEDS[2]);
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
  __disable_irq();
  float angle;
  angle = acos(time_offset01 * speed_of_sound / US_separations[0]) * rad_to_deg;
  if (time_offset02 > 100) {
    angle = -angle;
  }
  angle -= 90;
  __enable_irq();
  return angle;
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

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


  // Initialize timer
  error_code = nrfx_timer_init(&detection_timer, &timer_cfg, detection_timer_event_handler);
  APP_ERROR_CHECK(error_code);
  nrfx_timer_clear(&detection_timer);
  nrfx_timer_enable(&detection_timer);

  // Initialize app timer
  lfclk_request();
  error_code = app_timer_init();
  APP_ERROR_CHECK(error_code);
  create_app_timers();

  printf("All initializations success.\n");
  // loop forever
  while (1) {
    __WFI();
  }
}
