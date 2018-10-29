// servo_driver
// init example code

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "virtual_timer.h"

// sample toggle function
void led0_toggle() {
    nrf_gpio_pin_toggle(BUCKLER_LED0);
}

void pwm_up_counter_example(void) {
  // example code from nRF data sheet p496 shows the counter operating in up (MODE=PWM_MODE_Up) mode with three PWM channels with the same frequency but different duty cycle. The counter is automatically reset to zero when COUNTERTOP is reached and OUT[n] will invert. OUT[n] is held low if the compare value is 0 and held high respectively if set to COUNTERTOP given that the polarity is set to FallingEdge. Running in up counter mode will result in pulse widths that are edge-aligned. See the code example below:

  uint16_t pwm_seq[4] = {PWM_CH0_DUTY, PWM_CH1_DUTY, PWM_CH2_DUTY, PWM_CH3_DUTY};
  NRF_PWM0->PSEL.OUT[0] = (first_pin << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM0->PSEL.OUT[1] = (second_pin << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
  NRF_PWM0->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
  NRF_PWM0->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);
  NRF_PWM0->COUNTERTOP = (16000 << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
  NRF_PWM0->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
  NRF_PWM0->SEQ[0].PTR = ((uint32_t)(pwm_seq) << PWM_SEQ_PTR_PTR_Pos);
  NRF_PWM0->SEQ[0].CNT = ((sizeof(pwm_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
  NRF_PWM0->SEQ[0].REFRESH = 0;
  NRF_PWM0->SEQ[0].ENDDELAY = 0;
  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

int main(void) {
  // init code from lab
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Board initialized!\n");

  // set gipo pin (change BUCKLER_LED0 to desire pin)
  nrf_gpio_pin_dir_set(BUCKLER_LED0, NRF_GPIO_PIN_DIR_OUTPUT);

  // loop forever
  while (1) {
    nrf_delay_ms(1000);
  }
}
