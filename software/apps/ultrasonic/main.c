#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrf_drv_gpiote.h"

#include "buckler.h"
#include "gpiote.h"

// LED = pin 25, 24, 23 or can do BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2
#define US0_PIN BUCKLER_GROVE_A0
#define US1_PIN BUCKLER_GROVE_A1
#define US2_PIN BUCKLER_GROVE_D1


static uint32_t US_disable_interval = 35000;
static uint32_t US_detection_interval = 1000;

static uint32_t US_times[3] = {0, 0, 0};
static uint32_t time_offset01, time_offset02, time_offset12;

static int counts = 0;

// Read the current value of the timer counter
uint32_t read_timer(int channel) {
  // Should return the value of the internal counter for TIMER1
  NRF_TIMER1->TASKS_CAPTURE[channel] = 1;
  return NRF_TIMER1->CC[channel];
}

void calculate_time_offset(void) {
    printf("%d: US0: %d, US1: %d, US2: %d\n", counts, US_times[0], US_times[1], US_times[2]);
    counts += 1;
    // US_times[0] = 0;
    // US_times[1] = 0;
    // US_times[2] = 0;
}

void TIMER1_IRQHandler(void) {
  // turn on interrupt for US detectors
    if (NRF_TIMER1->EVENTS_COMPARE[0]) {
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;
        NRF_TIMER1->INTENCLR = 1 << 16;
        // NRF_GPIOTE->INTENSET = 1;
        nrfx_gpiote_in_event_enable(US0_PIN, true);
    }
    if (NRF_TIMER1->EVENTS_COMPARE[1]) {
        NRF_TIMER1->EVENTS_COMPARE[1] = 0;
        NRF_TIMER1->INTENCLR = 1 << 17;
        // NRF_GPIOTE->INTENSET = 1;
        nrfx_gpiote_in_event_enable(US1_PIN, true);
    }
    if (NRF_TIMER1->EVENTS_COMPARE[2]) {
        NRF_TIMER1->EVENTS_COMPARE[2] = 0;
        NRF_TIMER1->INTENCLR = 1 << 18;
        // NRF_GPIOTE->INTENSET = 1;
        nrfx_gpiote_in_event_enable(US2_PIN, true);
    }
    if (NRF_TIMER1->EVENTS_COMPARE[3]) {
        NRF_TIMER1->EVENTS_COMPARE[3] = 0;
        calculate_time_offset();
    }
}

void US0_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    // NRF_GPIOTE->INTENCLR = 1;
    nrfx_gpiote_in_event_disable(US0_PIN);
    US_times[0] = read_timer(0);
    NRF_TIMER1->INTENSET = 1 << 16;
    NRF_TIMER1->CC[0] = US_times[0] + US_disable_interval;
    NRF_TIMER1->CC[3] = US_times[0] + US_detection_interval;
    nrf_drv_gpiote_out_toggle(BUCKLER_LED0);
    printf("US0 triggered at %i\n", US_times[0]);
    // calculate_time_offset();
}

void US1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    // NRF_GPIOTE->INTENCLR = 1;
    nrfx_gpiote_in_event_disable(US1_PIN);
    US_times[1] = read_timer(1);
    NRF_TIMER1->INTENSET = 1 << 17;
    NRF_TIMER1->CC[1] = US_times[1] + US_disable_interval;
    NRF_TIMER1->CC[3] = US_times[1] + US_detection_interval;
    nrf_drv_gpiote_out_toggle(BUCKLER_LED1);
    printf("US1 triggered at %i\n", US_times[1]);
    // calculate_time_offset();
}

void US2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    // NRF_GPIOTE->INTENCLR = 1;
    nrfx_gpiote_in_event_disable(US2_PIN);
    US_times[2] = read_timer(2);
    NRF_TIMER1->INTENSET = 1 << 18;
    NRF_TIMER1->CC[2] = US_times[2] + US_disable_interval;
    NRF_TIMER1->CC[3] = US_times[2] + US_detection_interval;
    nrf_drv_gpiote_out_toggle(BUCKLER_LED2);
    printf("US2 triggered at %i\n", US_times[2]);
    // calculate_time_offset();
}

static void gpio_init(void) {
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
    // Init LEDs for debugging
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(BUCKLER_LED0, &out_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(BUCKLER_LED1, &out_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_out_init(BUCKLER_LED2, &out_config);
    APP_ERROR_CHECK(err_code);

    // Init ultrasonic
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(US0_PIN, &in_config, US0_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(US1_PIN, &in_config, US1_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(US2_PIN, &in_config, US2_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_event_enable(US0_PIN, true);
    nrfx_gpiote_in_event_enable(US1_PIN, true);
    nrfx_gpiote_in_event_enable(US2_PIN, true);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // Set LED for debugging
  // nrf_gpio_pin_dir_set(BUCKLER_LED0, NRF_GPIO_PIN_DIR_OUTPUT);
  // nrf_gpio_pin_dir_set(BUCKLER_LED1, NRF_GPIO_PIN_DIR_OUTPUT);
  // nrf_gpio_pin_dir_set(BUCKLER_LED2, NRF_GPIO_PIN_DIR_OUTPUT);

  // Initialize interrupts
  // NRF_GPIOTE->CONFIG[0] = MODE_EVENT | US0_PIN << PSEL_POS | POLARITY_LOTOHI << POLARITY_POS;
  // NRF_GPIOTE->CONFIG[1] = MODE_EVENT | US1_PIN << PSEL_POS | POLARITY_LOTOHI << POLARITY_POS;
  // NRF_GPIOTE->CONFIG[2] = MODE_EVENT | US2_PIN << PSEL_POS | POLARITY_LOTOHI << POLARITY_POS;
  // NRF_GPIOTE->INTENSET = 1;

  // NVIC_EnableIRQ(GPIOTE_IRQn);
  gpio_init();

  // Initialize timer
  NVIC_EnableIRQ(TIMER1_IRQn);
  
  NRF_TIMER1->BITMODE = 3;
  NRF_TIMER1->PRESCALER = 4;
  NRF_TIMER1->INTENSET = 1 << 19; // Enable compare for CC3

  NRF_TIMER1->TASKS_STOP = 1;
  NRF_TIMER1->TASKS_CLEAR = 1;
  NRF_TIMER1->TASKS_START = 1;

  // loop forever
  while (1) {
    // nrf_delay_ms(500);
    __WFI();
  }
}
