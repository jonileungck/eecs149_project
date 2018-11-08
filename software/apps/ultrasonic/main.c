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
#include "nrf_drv_timer.h"

#include "buckler.h"
#include "gpiote.h"

// LED = pin 25, 24, 23 or can do BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2
#define US0_PIN BUCKLER_GROVE_A0
#define US1_PIN BUCKLER_GROVE_A1
#define US2_PIN BUCKLER_GROVE_D1

// Ultrasonic time keeping
static const nrf_drv_timer_t US_timer = NRFX_TIMER_INSTANCE(1);
static const nrf_drv_timer_config_t timer_cfg = {
    .frequency          = NRF_TIMER_FREQ_1MHz,
    .mode               = NRF_TIMER_MODE_TIMER,
    .bit_width          = NRF_TIMER_BIT_WIDTH_32,
    .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
    .p_context          = NULL,
};

static uint32_t US_disable_interval = 1;
static uint32_t US_detection_interval = 0;

static uint32_t US_time[3] = {0, 0, 0};
static uint32_t time_offset01, time_offset02, time_offset12;

static int counts = 0;
// void GPIOTE_IRQHandler(void) {
//     uint32_t last_detection;
//     if (NRF_GPIOTE->EVENTS_IN[0]) {
//         NRF_GPIOTE->EVENTS_IN[0] = 0;
//         NRF_GPIOTE->INTENCLR = 1;
//         US_time[0] = nrfx_timer_capture(&US_timer, NRF_TIMER_CC_CHANNEL0);
//         last_detection = US_time[0];
//         nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL0, US_time[0] + US_disable_interval, 1);
//     }
//     if (NRF_GPIOTE->EVENTS_IN[1]) {
//         NRF_GPIOTE->EVENTS_IN[1] = 0;
//         NRF_GPIOTE->INTENCLR = 2;
//         US_time[1] = nrfx_timer_capture(&US_timer, NRF_TIMER_CC_CHANNEL1);
//         last_detection = US_time[1];
//         nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL1, US_time[1] + US_disable_interval, 1);
//     }
//     if (NRF_GPIOTE->EVENTS_IN[2]) {
//         NRF_GPIOTE->EVENTS_IN[2] = 0;
//         NRF_GPIOTE->INTENCLR = 4;
//         US_time[2] = nrfx_timer_capture(&US_timer, NRF_TIMER_CC_CHANNEL2);
//         last_detection = US_time[2];
//         nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL2, US_time[2] + US_disable_interval, 1);
//     }

//     // Delay before calculating time offset to make sure all 3 sensors have a chance to log detection.
//     nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL3, last_detection + US_detection_interval, 1);

//     //__disable_irq();
//     // calculate_time_offset();
//     //__enable_irq();

// }

void calculate_time_offset(void) {
    printf("%d: US0: %d, US1: %d, US2: %d\n", counts, US_time[0]/1000000, US_time[1]/1000000, US_time[2]/1000000);
    counts += 1;
    // US_time[0] = 0;
    // US_time[1] = 0;
    // US_time[2] = 0;
}

static void US_timer_event_handler(nrf_timer_event_t event_type, void* p_context) {
  // turn on interrupt
    printf("Timer event in\n");
    if (NRF_TIMER0->EVENTS_COMPARE[0]) {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        // NRF_GPIOTE->INTENSET = 1;
        printf("Timer US0\n");
        nrfx_gpiote_in_event_enable(US0_PIN, true);
    }
    if (NRF_TIMER1->EVENTS_COMPARE[1]) {
        NRF_TIMER1->EVENTS_COMPARE[1] = 0;
        // NRF_GPIOTE->INTENSET = 1 << 1;
        printf("Timer US1\n");
        nrfx_gpiote_in_event_enable(US1_PIN, true);
    }
    if (NRF_TIMER1->EVENTS_COMPARE[2]) {
        NRF_TIMER1->EVENTS_COMPARE[2] = 0;
        // NRF_GPIOTE->INTENSET = 1 << 2;
        printf("Timer US2\n");
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
    US_time[0] = nrfx_timer_capture(&US_timer, NRF_TIMER_CC_CHANNEL0);
    nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL0, US_time[0] + US_disable_interval, true);
    nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL3, US_time[0] + US_detection_interval, true);
    nrf_drv_gpiote_out_toggle(BUCKLER_LED0);
    printf("US0 triggered\n");
    // calculate_time_offset();
}

void US1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    // NRF_GPIOTE->INTENCLR = 1 << 1;
    nrfx_gpiote_in_event_disable(US1_PIN);
    US_time[1] = nrfx_timer_capture(&US_timer, NRF_TIMER_CC_CHANNEL1);
    nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL1, US_time[1] + US_disable_interval, true);
    nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL3, US_time[1] + US_detection_interval, true);
    nrf_drv_gpiote_out_toggle(BUCKLER_LED1);
    printf("US1 triggered\n");
    // calculate_time_offset();
}

void US2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    // NRF_GPIOTE->INTENCLR = 1 << 2;
    nrfx_gpiote_in_event_disable(US2_PIN);
    US_time[2] = nrfx_timer_capture(&US_timer, NRF_TIMER_CC_CHANNEL2);
    nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL2, US_time[2] + US_disable_interval, true);
    nrfx_timer_compare(&US_timer, NRF_TIMER_CC_CHANNEL3, US_time[2] + US_detection_interval, true);
    nrf_drv_gpiote_out_toggle(BUCKLER_LED2);
    printf("US2 triggered\n");
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
    // in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(US0_PIN, &in_config, US0_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(US1_PIN, &in_config, US1_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(US2_PIN, &in_config, US2_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(US0_PIN, true);
    nrf_drv_gpiote_in_event_enable(US1_PIN, true);
    nrf_drv_gpiote_in_event_enable(US2_PIN, true);
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
  error_code = nrfx_timer_init(&US_timer, &timer_cfg, US_timer_event_handler);
  APP_ERROR_CHECK(error_code);
  US_disable_interval = nrfx_timer_ms_to_ticks(&US_timer, 20);
  US_detection_interval = nrfx_timer_us_to_ticks(&US_timer, 1000);
  nrfx_timer_clear(&US_timer);
  nrfx_timer_enable(&US_timer);

  // loop forever
  while (1) {
    // nrf_delay_ms(500);
    __WFI();
  }
}
