/*
 * VL53L0X_user.h
 *
 *  Created on: Mar 11, 2025
 *      Author: Kirill
 */

#ifndef SRC_V53L0X_VL53L0X_USER_H_
#define SRC_V53L0X_VL53L0X_USER_H_

#include <stdio.h>
#include <stdint.h>

#define VL53L0X_MSG_BUFFER_SIZE 16

typedef struct {
	// data from statInfo_t_VL53L0X struct
	uint16_t rawDistance; //uncorrected distance  [mm],   uint16_t
	uint16_t signalCnt;   //Signal  Counting Rate [mcps], uint16_t, fixpoint9.7
	uint16_t ambientCnt;  //Ambient Counting Rate [mcps], uint16_t, fixpoint9.7
	uint16_t spadCnt;     //Effective SPAD return count,  uint16_t, fixpoint8.8
	uint8_t  rangeStatus; //Ranging status (0-15)

	uint8_t g_isTimeout;
	uint8_t g_stopVariable;
	uint32_t g_measTimBudUs;
	uint8_t deviceAddress;
	uint8_t msgBuffer[VL53L0X_MSG_BUFFER_SIZE];
	void (*i2c_receive) (uint8_t DevAddr, uint8_t* pData, uint16_t Size);
	void (*i2c_transmit) (uint8_t DevAddr, uint8_t* pData, uint16_t Size);
	void (*sleep_us) (uint32_t us);
} VL53L0X_t;

/* API */
int VL53L0X_init(VL53L0X_t* self, uint8_t deviceAddress);
void VL53L0X_reg_callbacks(
		VL53L0X_t* self,
		void (*receive_cb) (uint8_t DevAddr, uint8_t* pData, uint16_t Size),
		void (*transmit_cb) (uint8_t DevAddr, uint8_t* pData, uint16_t Size),
		void (*sleep_us_cb) (uint32_t us)
		);

void VL53L0X_setup(VL53L0X_t* self, float signalRateLimitMcps);
uint32_t VL53L0X_GetMeasurementTimingBudget(VL53L0X_t* self);
uint8_t VL53L0X_SetMeasurementTimingBudget(VL53L0X_t* self, uint32_t budget_us);
uint8_t VL53L0X_SetVcselPulsePeriod(VL53L0X_t* self, uint8_t type, uint8_t period_pclks);
void VL53L0X_start_continuous(VL53L0X_t* self, uint32_t period_ms);
void VL53L0X_stop_continuous(VL53L0X_t* self);
uint16_t VL53L0X_read_continuous_mm(VL53L0X_t* self);
uint16_t VL53L0X_read_single_mm(VL53L0X_t* self);
/* End API */

#endif /* SRC_V53L0X_VL53L0X_USER_H_ */
