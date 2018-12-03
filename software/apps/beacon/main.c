// Code based on Button and Switch on the Buckler repo

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
#define NUM_US 4

static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};
static uint8_t US[NUM_US] = {BUCKLER_GROVE_A0, BUCKLER_GROVE_D0, BUCKLER_SENSORS_SCL, BUCKLER_UART_TX};

void set_US_pins(void) {
  for (int i=0; i<NUM_US; i++) {
    nrfx_gpiote_out_set(US[i]);
  }
}

void clear_US_pins(void) {
  for (int i=0; i<NUM_US; i++) {
    nrfx_gpiote_out_clear(US[i]);
  }
}

// handler called whenever an input pin changes
void pin_change_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  switch(pin) {
    case BUCKLER_BUTTON0: {
      if (nrfx_gpiote_in_is_set(BUCKLER_BUTTON0)) {
        nrfx_gpiote_out_set(LEDS[0]);
        clear_US_pins();
      } else {
        nrfx_gpiote_out_clear(LEDS[0]);
        set_US_pins();
      }
      break;
    }

    case BUCKLER_SWITCH0: {
      if (nrfx_gpiote_in_is_set(BUCKLER_SWITCH0)) {
        nrfx_gpiote_out_set(LEDS[0]);
        nrfx_gpiote_out_set(LEDS[1]);
        nrfx_gpiote_out_clear(LEDS[2]);
      } else {
        nrfx_gpiote_out_clear(LEDS[1]);
        nrfx_gpiote_out_set(LEDS[2]);
      }
      break;
    }
  }
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize power management
  error_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(error_code);

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

  // configure button and switch
  // input pin, trigger on either edge, low accuracy (allows low-power operation)
  nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
  in_config.pull = NRF_GPIO_PIN_NOPULL;
  error_code = nrfx_gpiote_in_init(BUCKLER_BUTTON0, &in_config, pin_change_handler);
  nrfx_gpiote_in_event_enable(BUCKLER_BUTTON0, true);

  in_config.pull = NRF_GPIO_PIN_NOPULL;
  error_code = nrfx_gpiote_in_init(BUCKLER_SWITCH0, &in_config, pin_change_handler);
  nrfx_gpiote_in_event_enable(BUCKLER_SWITCH0, true);


  // configure ultrasonic transmitter
  for (int i=0; i<NUM_US; i++) {
    error_code = nrfx_gpiote_out_init(US[i], &out_config);
    APP_ERROR_CHECK(error_code);
  }

  // set initial states for LEDs
  nrfx_gpiote_out_set(LEDS[0]);
  if (nrfx_gpiote_in_is_set(BUCKLER_SWITCH0)) {
    nrfx_gpiote_out_set(LEDS[1]);
    nrfx_gpiote_out_clear(LEDS[2]);
  } else {
    nrfx_gpiote_out_clear(LEDS[1]);
    nrfx_gpiote_out_set(LEDS[2]);
  }

  // loop forever
  while (1) {
    if (nrfx_gpiote_in_is_set(BUCKLER_SWITCH0)) {
      __WFI();
    } else {
      nrf_gpio_pin_clear(BUCKLER_LED0);
      set_US_pins();
      nrf_delay_ms(1);
      clear_US_pins();
      nrf_delay_ms(100);
      nrf_gpio_pin_set(BUCKLER_LED0);
      nrf_delay_ms(899);
    }
  }
}

