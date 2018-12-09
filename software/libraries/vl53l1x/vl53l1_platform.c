
/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/


#include "vl53l1_platform.h"
// #include "vl53l1_platform_log.h"
#include "vl53l1_api.h"

// #include "stm32xxx_hal.h"
#include <string.h>
#include <time.h>
// #include <pthread.h>
// #include <unistd.h>
// #include <math.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nrf_twi_mngr.h"

// static pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER; 

#define TWI_BUFFER_SIZE    6


// #define I2C_TIME_OUT_BASE   10
// #define I2C_TIME_OUT_BYTE   1

// #ifdef VL53L1_LOG_ENABLE
// #define trace_print(level, ...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_PLATFORM, level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)
// #define trace_i2c(...) VL53L1_trace_print_module_function(VL53L1_TRACE_MODULE_NONE, VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)
// #endif

// #ifndef HAL_I2C_MODULE_ENABLED
// #warning "HAL I2C module must be enable "
// #endif

//extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_pI2cHandle    (&hi2c1)

/* when not customized by application define dummy one */
// #ifndef VL53L1_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_GetI2cBus(...) (void)0
// #endif

// #ifndef VL53L1_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
// #   define VL53L1_PutI2cBus(...) (void)0
// #endif

// uint8_t _I2CBuffer[256];

// int _I2CWrite(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//     int status = 0;
//     return status;
// }

// int _I2CRead(VL53L1_DEV Dev, uint8_t *pdata, uint32_t count) {
//    int status = 0;
//    return Status;
// }
static const nrf_twi_mngr_t* i2c_manager = NULL;
uint8_t twi_tx_buffer[TWI_BUFFER_SIZE];

void tof_init(const nrf_twi_mngr_t* i2c) {
	i2c_manager = i2c;
}

static unsigned int usleep(unsigned int useconds) {
    nrf_delay_ms(useconds / 1000);
    return useconds;
}

static void merge_register_and_data(uint8_t* new_buffer, uint16_t reg, uint8_t* p_data, uint8_t length) {
  new_buffer[0] = reg >> 8;
  new_buffer[1] = reg & 0xFF;
  memcpy((new_buffer + 2), p_data, length);
}

static void i2c_read_one_byte_func(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *p_data, uint8_t length) {
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ(i2c_addr, p_data, length, 0),
  };
  ret_code_t error_code = nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  APP_ERROR_CHECK(error_code);
}

static void i2c_write_one_byte_func(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *p_data, uint8_t length) {
  // Merging register address and p_data into one buffer
  uint8_t buf[length+1];
  buf[0] = reg_addr;
  memcpy((buf + 1), p_data, length);
  // Transferring
  nrf_twi_mngr_transfer_t const write_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, buf, length + 1, 0),
  };
  ret_code_t error_code = nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL); 
  APP_ERROR_CHECK(error_code);
}

static void i2c_read_func(uint8_t i2c_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t length) {
  uint8_t w_addr[2];
  w_addr[0] = reg_addr >> 8;
  w_addr[1] = reg_addr & 0xFF;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, w_addr, 2, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ(i2c_addr, p_data, length, 0),
  };
  ret_code_t error_code = nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  APP_ERROR_CHECK(error_code);
}

static void i2c_write_func(uint8_t i2c_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t length) {
  // Merging register address and p_data into one buffer
  uint8_t buf[length+2];
  merge_register_and_data(buf, reg_addr, p_data, length);

  // Transferring
  nrf_twi_mngr_transfer_t const write_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, buf, length + 2, 0),
  };
  ret_code_t error_code = nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL); 
  APP_ERROR_CHECK(error_code);
}

static int i2c_write(VL53L1_DEV Dev, uint16_t cmd,
                    uint8_t *data, uint8_t len)
{
    int result = VL53L1_ERROR_NONE;
    if (Dev->TCA9548A_Device < 8) {
      // pthread_mutex_lock(&i2c_mutex);
      
      i2c_write_one_byte_func(Dev->TCA9548A_Address, (1 << Dev->TCA9548A_Device), NULL, 0);
      i2c_write_func(Dev->I2cDevAddr, cmd, data, len);
      // pthread_mutex_unlock(&i2c_mutex);
    }
    return result;
}

static int i2c_read(VL53L1_DEV Dev, uint16_t cmd,
                    uint8_t * data, uint8_t len)
{
    int result = VL53L1_ERROR_NONE;
    if (Dev->TCA9548A_Device < 8) {
      // pthread_mutex_lock(&i2c_mutex);
      
      i2c_write_one_byte_func(Dev->TCA9548A_Address, (1 << Dev->TCA9548A_Device), NULL, 0);
      i2c_read_func(Dev->I2cDevAddr, cmd, data, len);
      // pthread_mutex_unlock(&i2c_mutex);
    }
    return result;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
	return i2c_write(pdev, index, pdata, count);

	//VL53L1_Error Status = VL53L1_ERROR_NONE;
	//return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
	return i2c_read(pdev, index, pdata, count);

	//VL53L1_Error Status = VL53L1_ERROR_NONE;
	//return Status;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV pdev, uint16_t index, uint8_t data) {
	return i2c_write(pdev, index, &data, 1);

	//VL53L1_Error Status = VL53L1_ERROR_NONE;
	//return Status;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV pdev, uint16_t index, uint16_t data) {
	uint8_t buf[4];
	buf[1] = data>>0&0xFF;
	buf[0] = data>>8&0xFF;
	return i2c_write(pdev, index, buf, 2);

	//VL53L1_Error Status = VL53L1_ERROR_NONE;
	//return Status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV pdev, uint16_t index, uint32_t data) {
	uint8_t buf[4];
	buf[3] = data>>0&0xFF;
	buf[2] = data>>8&0xFF;
	buf[1] = data>>16&0xFF;
	buf[0] = data>>24&0xFF;
	return i2c_write(pdev, index, buf, 4);

	//VL53L1_Error Status = VL53L1_ERROR_NONE;
	//return Status;
}

VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV pdev, uint16_t index, uint8_t AndData, uint8_t OrData) {
	int32_t status_int;
	uint8_t data;

	status_int = i2c_read(pdev, index, &data, 1);

	if (status_int != 0)
	{
		return  status_int;
	}

	data = (data & AndData) | OrData;
	return i2c_write(pdev, index, &data, 1);

	//VL53L1_Error Status = VL53L1_ERROR_NONE;
	//return Status;
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV pdev, uint16_t index, uint8_t *data) {
	uint8_t tmp = 0;
	int ret = i2c_read(pdev, index, &tmp, 1);
	*data = tmp;
  // printf("read byte: %hhu\n", tmp);
	return ret;

	//VL53L1_Error Status = VL53L1_ERROR_NONE;
	//return Status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV pdev, uint16_t index, uint16_t *data) {
	uint8_t buf[2];
	int ret = i2c_read(pdev, index, buf, 2);
	uint16_t tmp = 0;
	tmp |= buf[1]<<0;
	tmp |= buf[0]<<8;
	*data = tmp;
	return ret;

	//VL53L1_Error Status = VL53L1_ERROR_NONE;
	//return Status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV pdev, uint16_t index, uint32_t *data) {
	uint8_t buf[4];
	int ret = i2c_read(pdev, index, buf, 4);
	uint32_t tmp = 0;
	tmp |= buf[3]<<0;
	tmp |= buf[2]<<8;
	tmp |= buf[1]<<16;
	tmp |= buf[0]<<24;
	*data = tmp;
	return ret;

	//VL53L1_Error Status = VL53L1_ERROR_NONE;
	//return Status;
}

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

//#define trace_print(level, ...) 
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_PLATFORM, 
//	level, VL53L1_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

//#define trace_i2c(...) 
//	_LOG_TRACE_PRINT(VL53L1_TRACE_MODULE_NONE, 
//	VL53L1_TRACE_LEVEL_NONE, VL53L1_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	VL53L1_Error status  = VL53L1_ERROR_NONE;
	return status;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_DEV pdev, int32_t wait_ms){
	usleep(wait_ms * 1000);
    return VL53L1_ERROR_NONE;
	// VL53L1_Error status  = VL53L1_ERROR_NONE;
	// return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_DEV pdev, int32_t wait_us){
	usleep(wait_us);
    return VL53L1_ERROR_NONE;
	// VL53L1_Error status  = VL53L1_ERROR_NONE;
	// return status;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_DEV pdev,
	uint32_t   timeout_ms,
	uint16_t   index,
	uint8_t	   value,
	uint8_t	   mask,
	uint32_t   poll_delay_ms)
{
	uint8_t  register_value = 0;

	VL53L1_Error status  = VL53L1_ERROR_NONE;

	int32_t attempts = timeout_ms / poll_delay_ms;

	for(int32_t x = 0; x < attempts; x++){
		status = VL53L1_RdByte(
					pdev,
					index,
					&register_value);
		if (status == VL53L1_ERROR_NONE && (register_value & mask) == value) {
			return VL53L1_ERROR_NONE;
		}
		usleep(poll_delay_ms * 1000);
	}

	return VL53L1_ERROR_TIME_OUT;
	// VL53L1_Error status  = VL53L1_ERROR_NONE;
	// return status;
}


