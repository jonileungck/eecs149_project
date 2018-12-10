// Our master beacon-following robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"
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

#include "nrf_drv_pwm.h"
#include "nrf_timer.h"
#include "boards.h"

#include "vl53l1_api.h"
#include "vl53l1_platform.h"
#include "vl53l1_register_settings.h"

#include "app_util.h"
#include "simple_ble.h"

//BLE section start
// Create a timer
APP_TIMER_DEF(adv_timer);

// BLE configuration
static simple_ble_config_t ble_config = {
        // BLE address is c0:98:e5:49:00:01
        .platform_id       = 0x4A,    // used as 4th octet in device BLE address
        .device_id         = 0x0001,  // used as the 5th and 6th octet in the device BLE address, you will need to change this for each device you have
        .adv_name          = "EE149", // irrelevant in this example
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS), // send a packet once per second (minimum is 20 ms)
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS), // irrelevant if advertising only
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS), // irrelevant if advertising only
};
simple_ble_app_t* simple_ble_app;

// Sends the specified data over BLE advertisements
void set_ble_payload(uint8_t* buffer, uint8_t length) {
  static uint8_t adv_buffer[24] = {0};
  static ble_advdata_manuf_data_t adv_payload = {
    .company_identifier = 0x02E0, // Lab11 company ID (University of Michigan)
    .data.p_data = adv_buffer,
    .data.size = 24,
  };

  // copy over up to 23 bytes of advertisement payload
  adv_buffer[0] = 0x23; // identifies a Buckler advertisement payload
  if (length > 23) {
    length = 23; // maximum size is 23 bytes of payload
  }
  memcpy(&(adv_buffer[1]), buffer, length);
  adv_payload.data.size = 1+length;

  // create an advertisement with a manufacturer-specific data payload
  // https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.0.0%2Fstructble__advdata__t.html
  ble_advdata_t advdata = {0};
  advdata.name_type = BLE_ADVDATA_NO_NAME; // do not include device name (adv_name) in advertisement
  advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; // BLE Low energy advertisement
  advdata.p_manuf_specific_data = &adv_payload;

  // update advertisement data and start advertising
  simple_ble_set_adv(&advdata, NULL);
}
//BLE section end

// titlis's section - start
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

static volatile uint8_t distance_data[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
void pull_distance_data()
{
    //test
}


void pwm_update_duty_cycle(double duty_cycle)
{
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
void TIMER3_IRQHandler(void) {
    if(mode == 0) {
        pwm_update_duty_cycle(2.3);
    } else if(mode == 2) {
        pwm_update_duty_cycle(6.8);
    }
    //Read tof sensor data here
    pull_distance_data();
    //Send data using BLE
    //set_ble_payload(buffer, 23);

    nrf_timer_event_clear(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE0);
    nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_CLEAR);
    mode = (mode + 1) % 4;
}

void servo_start(void) {
    pwm_init();
    NVIC_EnableIRQ(TIMER3_IRQn);
    nrf_timer_mode_set(NRF_TIMER3,NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(NRF_TIMER3, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_frequency_set(NRF_TIMER3, NRF_TIMER_FREQ_1MHz);
    //timer for every 250ms
    nrf_timer_cc_write(NRF_TIMER3, NRF_TIMER_CC_CHANNEL0, 1000000/4);
    nrf_timer_int_enable(NRF_TIMER3,NRF_TIMER_INT_COMPARE0_MASK );
    nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_START);
}

void buckler_init() {
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

    //ble start
    simple_ble_app = simple_ble_init(&ble_config);
    // set_ble_payload(buffer, 23);
    //ble end
}
// titlis's section - end

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

// =========== ToF part starts here ===============

static VL53L1_Dev_t dev_0;
static VL53L1_Dev_t dev_1;
static VL53L1_Dev_t dev_2;
static VL53L1_Dev_t dev_3;

static VL53L1_DEV Dev_0 = &dev_0;
static VL53L1_DEV Dev_1 = &dev_1;
static VL53L1_DEV Dev_2 = &dev_2;
static VL53L1_DEV Dev_3 = &dev_3;

static VL53L1_RangingMeasurementData_t RangingMeasurementData;
static VL53L1_RangingMeasurementData_t *pRangingMeasurementData = &RangingMeasurementData;

static VL53L1_Error status;

void vl53l1_init(VL53L1_DEV Dev, uint8_t I2cDevAddr, uint8_t TCA9548A_Addr, uint8_t TCA9548A_Device) {
  Dev->I2cDevAddr = VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT;
  Dev->TCA9548A_Address = TCA9548A_Addr;
  Dev->TCA9548A_Device = TCA9548A_Device;

  printf("Initializing sensor\n");
  status = VL53L1_software_reset(Dev);
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);  // [20, 100]ms
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 250);  // min = budget + 4ms
  status = VL53L1_StartMeasurement(Dev);
  // printf("status: %hhu\n", status);
}


uint16_t ranging(VL53L1_DEV Dev) {
  status = VL53L1_WaitMeasurementDataReady(Dev);
  if (!status) {
    status = VL53L1_GetRangingMeasurementData(Dev, pRangingMeasurementData);
    if (status == 0) {
      printf("Ranging status: %hhu\n", pRangingMeasurementData->RangeStatus);
      printf("RangeMilliMeter: %hu\n", pRangingMeasurementData->RangeMilliMeter);
      printf("SignalRateRtnMegaCps: %f\n", pRangingMeasurementData->SignalRateRtnMegaCps/65536.0);
      printf("AmbientRateRtnMegaCps: %f\n", pRangingMeasurementData->AmbientRateRtnMegaCps/65336.0);
      status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
      return pRangingMeasurementData->RangeMilliMeter;
    } else {
      return 0;
    }
  } else {
    printf("error waiting for data ready: %hhu\n", status);
    return 0;
  }
}


void i2c_init() {
  ret_code_t error_code = NRF_SUCCESS;
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
}


void read_tof(uint8_t* data) {
  uint16_t measurement;
  measurement = ranging(Dev_0);
  data[0] = (uint8_t) (measurement >> 4);
  measurement = ranging(Dev_1);
  data[1] = (uint8_t) (measurement >> 4);
  measurement = ranging(Dev_2);
  data[2] = (uint8_t) (measurement >> 4);
  measurement = ranging(Dev_3);
  data[3] = (uint8_t) (measurement >> 4);
}

// ============ ToF part ends here =================

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
    const float CONVERSION = 0.00008529;
    float retval = 0;
    float retval_edge_case = 0;
    retval = (current_encoder - previous_encoder)*CONVERSION;

    if (current_encoder < previous_encoder) {
        retval_edge_case = (0xFFFF - previous_encoder + current_encoder)*CONVERSION;
        if (retval_edge_case > 1) {
            return 0;
        }
        return retval_edge_case;
    }

    return retval;

}

int main(void) {
  buckler_init();
  servo_start();
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
  tof_init(&twi_mngr_instance);


  // initialize tof sensor
  vl53l1_init(Dev_0, VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT, 0x70, 0);
  vl53l1_init(Dev_1, VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT, 0x70, 1);
  vl53l1_init(Dev_2, VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT, 0x70, 2);
  vl53l1_init(Dev_3, VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT, 0x70, 3);

  if(status) {
    printf("VL53L1_StartMeasurement failed\n");
    while(1);
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

  uint8_t data[4];

  // loop forever, running state machine
  while (1) {
    //BLE
    power_manage();
    // read sensors from robot
    kobukiSensorPoll(&sensors);
    bump_left = sensors.bumps_wheelDrops.bumpLeft;
    bump_right = sensors.bumps_wheelDrops.bumpRight | sensors.bumps_wheelDrops.bumpCenter;
    right_whl_encoder_curr = sensors.rightWheelEncoder;
    distance = measure_distance(right_whl_encoder_prev, right_whl_encoder_curr);

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(100);

    // get measurements
    read_tof(data);
    printf("Sensor 0 16bit reading: %hu\n", data[0] << 4);
    printf("Sensor 1 16bit reading: %hu\n", data[1] << 4);
    printf("Sensor 2 16bit reading: %hu\n", data[2] << 4);
    printf("Sensor 3 16bit reading: %hu\n", data[3] << 4);

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
            distance = 0;
            direction = false;
            target_angle = -45;
            current_angle = 0;
            mpu9250_start_gyro_integration();
            state = AVOIDL;
        } else if (bump_right) {
            right_whl_encoder_prev = right_whl_encoder_curr;
            distance = 0;
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
        snprintf(print_str, 16, "%f", current_angle);
        display_write(print_str, DISPLAY_LINE_1);
        if (is_button_pressed(&sensors)) {
          target_angle = 0;
          current_angle = 0;
          distance = 0;
          direction = true;
          mpu9250_stop_gyro_integration();
          state = OFF;
        } else {
            if (distance < 0.1 && !direction) {
                kobukiDriveDirect(-100, -100);
            } else if (current_angle > target_angle) {
                current_angle = mpu9250_read_gyro_integration().z_axis;
                direction = true;
                distance = 0;
                kobukiDriveDirect(100, -100);
            } else if (distance < 0.5) {
                kobukiDriveDirect(100, 100);
            } else {
                target_angle = 0;
                current_angle = 0;
                distance = 0;
                mpu9250_stop_gyro_integration();
                state = DRIVING;
            }
        }
        break;
      }

      case AVOIDR: {
        display_write("AVOIDR", DISPLAY_LINE_0);
        snprintf(print_str, 16, "%f", current_angle);
        display_write(print_str, DISPLAY_LINE_1);
        if (is_button_pressed(&sensors)) {
          target_angle = 0;
          current_angle = 0;
          distance = 0;
          direction = true;
          mpu9250_stop_gyro_integration();
          state = OFF;
        } else {
            if (distance < 0.1 && !direction) {
                kobukiDriveDirect(-100, -100);
            } else if (current_angle < target_angle) {
                current_angle = mpu9250_read_gyro_integration().z_axis;
                direction = true;
                distance = 0;
                kobukiDriveDirect(100, -100);
            } else if (distance < 0.5) {
                kobukiDriveDirect(100, 100);
            } else {
                target_angle = 0;
                current_angle = 0;
                distance = 0;
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
