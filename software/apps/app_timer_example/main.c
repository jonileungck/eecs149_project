#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"

#include "app_timer.h"
#include "nrf_drv_clock.h"

// General application timer settings.
#define APP_TIMER_OP_QUEUE_SIZE         3     // Size of timer operation queues.

APP_TIMER_DEF(m_led_b_timer_id);

// LED array
static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};

// handler called whenever an input pin changes
void pin_change_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  if (!nrfx_gpiote_in_is_set(BUCKLER_BUTTON0)) {
    printf("Button pressed.\n");
    // Start single shot timer. Increase the timeout with 1 second every time.
    ret_code_t err_code = app_timer_start(m_led_b_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
  } 
}

// Function starting the internal LFCLK oscillator.
// This is needed by RTC1 which is used by the application timer
// (When SoftDevice is enabled the LFCLK is always running and this is not needed).
static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

// Timeout handler for the repeated timer
static void timer_b_handler(void * p_context)
{
  printf("Handler called\n");
  nrf_gpio_pin_toggle(LEDS[0]);
}

// Create timers
static void create_timers()
{   
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&m_led_b_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_b_handler);
    APP_ERROR_CHECK(err_code);
}


int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize GPIO driver
  if (!nrfx_gpiote_is_init()) {
    error_code = nrfx_gpiote_init();
  }
  APP_ERROR_CHECK(error_code);

  // configure leds
  // manually-controlled (simple) output, initially set
  nrfx_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
  for (int i=0; i<3; i++) {
    error_code = nrfx_gpiote_out_init(LEDS[i], &out_config);
    APP_ERROR_CHECK(error_code);
  }

  // configure button
  // input pin, trigger on either edge, low accuracy (allows low-power operation)
  nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
  in_config.pull = NRF_GPIO_PIN_NOPULL;
  error_code = nrfx_gpiote_in_init(BUCKLER_BUTTON0, &in_config, pin_change_handler);
  nrfx_gpiote_in_event_enable(BUCKLER_BUTTON0, true);
  APP_ERROR_CHECK(error_code);

  // Initialize app timer
  lfclk_request();
  error_code = app_timer_init();
  APP_ERROR_CHECK(error_code);

  // Create application timers.
  create_timers();

  // loop forever
  while (1) {
    
  }
}

