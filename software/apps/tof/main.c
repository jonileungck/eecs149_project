// VL53L1X app

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrf_twi_mngr.h"
#include "nrfx_gpiote.h"

#include "buckler.h"
#include "vl53l1_api.h"
#include "vl53l1_platform.h"
#include "vl53l1_register_settings.h"


static VL53L1_Dev_t dev_0;
static VL53L1_DEV Dev_0 = &dev_0;

static VL53L1_Dev_t dev_1;
static VL53L1_DEV Dev_1 = &dev_1;

static VL53L1_Dev_t dev_2;
static VL53L1_DEV Dev_2 = &dev_2;

static VL53L1_Dev_t dev_3;
static VL53L1_DEV Dev_3 = &dev_3;

static VL53L1_RangingMeasurementData_t RangingMeasurementData;
static VL53L1_RangingMeasurementData_t *pRangingMeasurementData = &RangingMeasurementData;

static VL53L1_Error status;
static ret_code_t error_code;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

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
  printf("status: %hhu\n", status);
}

uint16_t ranging(VL53L1_DEV Dev) {
  status = VL53L1_WaitMeasurementDataReady(Dev);
  if (!status) {
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
  printf("===========SENSOR 0===========\n");
  measurement = ranging(Dev_0);
  data[0] = (uint8_t) (measurement >> 4);
  printf("===========SENSOR 1===========\n");
  measurement = ranging(Dev_1);
  data[1] = (uint8_t) (measurement >> 4);
  printf("===========SENSOR 2===========\n");
  measurement = ranging(Dev_2);
  data[2] = (uint8_t) (measurement >> 4);
  printf("===========SENSOR 3===========\n");
  measurement = ranging(Dev_3);
  data[3] = (uint8_t) (measurement >> 4);
}

int main(void) {
  // initialize RTT library
  ret_code_t error_code = NRF_SUCCESS;
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized\n");
  
  uint8_t data[4];

  // initialize i2c master (two wire interface)
  i2c_init();
  
  // link the i2c manager to the vl53l1x platform i2c manager
  tof_init(&twi_mngr_instance);

  // initialize tof sensor
  vl53l1_init(Dev_0, VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT, 0x70, 0);
  vl53l1_init(Dev_1, VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT, 0x70, 1);
  vl53l1_init(Dev_2, VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT, 0x70, 2);
  vl53l1_init(Dev_3, VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT, 0x70, 3);

  if(status)
  {
    printf("VL53L1_StartMeasurement failed\n");
    while(1);
  }

   while (1) {
    // get measurements
    read_tof(data);
    printf("Sensor 0 16bit reading: %hu\n", data[0] << 4);
    printf("Sensor 1 16bit reading: %hu\n", data[1] << 4);
    printf("Sensor 2 16bit reading: %hu\n", data[2] << 4);
    printf("Sensor 3 16bit reading: %hu\n", data[3] << 4);
    nrf_delay_ms(1000);
  }
}

