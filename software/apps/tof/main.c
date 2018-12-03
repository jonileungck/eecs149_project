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


static VL53L1_Dev_t dev;
static VL53L1_DEV Dev = &dev;

static VL53L1_RangingMeasurementData_t RangingMeasurementData;
static VL53L1_RangingMeasurementData_t *pRangingMeasurementData = &RangingMeasurementData;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  VL53L1_Error status;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);

  // initialize tof sensor
  Dev->I2cDevAddr = VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT;
  tof_init(&twi_mngr_instance);
  VL53L1_software_reset(Dev);
  printf("Autonomous Ranging Test\n");
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 500); 
  status = VL53L1_StartMeasurement(Dev);
  printf("status: %hhu\n", status);

  if(status)
  {
    printf("VL53L1_StartMeasurement failed\n");
    while(1);
  }

   while (1) {
    // get measurements
    status = VL53L1_WaitMeasurementDataReady(Dev);
    if (!status) {
      status = VL53L1_GetRangingMeasurementData(Dev, pRangingMeasurementData);
      if (status == 0) {
        printf("Ranging status: %hhu\n", pRangingMeasurementData->RangeStatus);
        printf("RangeMilliMeter: %hu\n", pRangingMeasurementData->RangeMilliMeter);
        printf("SignalRateRtnMegaCps: %f\n", pRangingMeasurementData->SignalRateRtnMegaCps/65536.0);
        printf("AmbientRateRtnMegaCps: %f\n", pRangingMeasurementData->AmbientRateRtnMegaCps/65336.0);
      }
      status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
    } else {
      printf("error waiting for data ready: %hhu\n", status);
    }
    nrf_delay_ms(100);
  }
  
}

