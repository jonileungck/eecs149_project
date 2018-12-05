// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"

#include "buckler.h"

#include "nrf_drv_pwm.h"
#include "nrf_timer.h"
#include "boards.h"


#define OUTPUT_PIN 16

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

// Declare variables holding PWM sequence values. In this example only one channel is used
nrf_pwm_values_individual_t seq_values[] = {0, 0, 0, 0};
nrf_pwm_sequence_t const seq =
{
    .values.p_individual = seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats         = 0,
    .end_delay       = 0
};


// Set duty cycle between 0 and 100%
void pwm_update_duty_cycle(double duty_cycle)
{

    // Check if value is outside of range. If so, set to 100%
    seq_values->channel_0 = 2500-(int)(duty_cycle * 25);

    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void pwm_init(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            OUTPUT_PIN, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_125kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 2500,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    // Init PWM without error handler
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
}

int mode = 0;
void TIMER2_IRQHandler(void) {
    if(mode == 0) {
      pwm_update_duty_cycle(5.5);
    } else if(mode == 2) {
      pwm_update_duty_cycle(10);
    }
    nrf_timer_event_clear(NRF_TIMER2, NRF_TIMER_EVENT_COMPARE0);
    nrf_timer_task_trigger(NRF_TIMER2, NRF_TIMER_TASK_CLEAR);
    mode = (mode + 1) % 4;
}

void servo_start(void) {
    pwm_init();
    NVIC_EnableIRQ(TIMER2_IRQn);
    nrf_timer_mode_set(NRF_TIMER2,NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(NRF_TIMER2, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_frequency_set(NRF_TIMER2, NRF_TIMER_FREQ_1MHz);
    //timer for every 500ms
    nrf_timer_cc_write(NRF_TIMER2, NRF_TIMER_CC_CHANNEL0, 1000000/4);
    nrf_timer_int_enable(NRF_TIMER2,NRF_TIMER_INT_COMPARE0_MASK );
    nrf_timer_task_trigger(NRF_TIMER2, NRF_TIMER_TASK_START);
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
  error_code = nrfx_gpiote_out_init(OUTPUT_PIN, &out_config);
  APP_ERROR_CHECK(error_code);

  // Start clock for accurate frequencies
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    // Wait for clock to start
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        ;

    servo_start();
}
