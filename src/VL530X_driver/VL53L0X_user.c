/*
 * VL53L0X_user.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Kirill
 */

#include "VL53L0X_user.h"
#include <string.h>

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

static const uint8_t I2C_READ = 1;
static const uint8_t I2C_WRITE = 0;
static const uint16_t timeoutUsMax = 2000;
// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection
typedef struct {
  uint8_t tcc, msrc, dss, pre_range, final_range;
}SequenceStepEnables;

typedef struct {
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
}SequenceStepTimeouts;

typedef enum { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;

typedef enum {
  SYSRANGE_START                              = 0x00,

  SYSTEM_THRESH_HIGH                          = 0x0C,
  SYSTEM_THRESH_LOW                           = 0x0E,

  SYSTEM_SEQUENCE_CONFIG                      = 0x01,
  SYSTEM_RANGE_CONFIG                         = 0x09,
  SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

  SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

  GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

  SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

  RESULT_INTERRUPT_STATUS                     = 0x13,
  RESULT_RANGE_STATUS                         = 0x14,

  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
  RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
  RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

  ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

  I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

  MSRC_CONFIG_CONTROL                         = 0x60,

  PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
  PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
  PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

  FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
  FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

  PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
  PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

  PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

  SYSTEM_HISTOGRAM_BIN                        = 0x81,
  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
  HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

  FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

  MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

  SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
  IDENTIFICATION_MODEL_ID                     = 0xC0,
  IDENTIFICATION_REVISION_ID                  = 0xC2,

  OSC_CALIBRATE_VAL                           = 0xF8,

  GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

  GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
  DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
  POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

  ALGO_PHASECAL_LIM                           = 0x30,
  ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
} regAddr;

static void _i2c_empty(uint8_t DevAddr, uint8_t* pData, uint16_t Size) {}
static void _sleep_default(uint32_t us) { int i; for (i = 0; i < 100 * us; i++); }

/* Private R/W functions*/
static void i2c_write(VL53L0X_t* self, uint8_t reg, uint8_t const *src, uint8_t size)
{
	// clear buffer
	memset(self->msgBuffer, 0, sizeof(VL53L0X_MSG_BUFFER_SIZE));
	// fill buffer
	memcpy(self->msgBuffer + 0, &reg, 1);
	memcpy(self->msgBuffer + 1, src, size);
	// send data
	self->i2c_transmit(self->deviceAddress | I2C_WRITE, self->msgBuffer, size + 1);
}

static void i2c_write8bit(VL53L0X_t* self, uint8_t reg, uint8_t value)
{
	i2c_write(self, reg, (uint8_t*)&value, 1);
}

static void i2c_write16bit(VL53L0X_t* self, uint8_t reg, uint16_t value)
{
	i2c_write(self, reg, (uint8_t*)&value, 2);
}

static void i2c_write32bit(VL53L0X_t* self, uint8_t reg, uint32_t value)
{
	i2c_write(self, reg, (uint8_t*)&value, 4);
}

static void i2c_read(VL53L0X_t* self, uint8_t reg, uint8_t *dst, uint8_t size)
{
	self->i2c_transmit(self->deviceAddress | I2C_WRITE, &reg, 1);
	self->i2c_receive(self->deviceAddress | I2C_READ, dst, size);
}

static uint8_t i2c_read8bit(VL53L0X_t* self, uint8_t reg)
{
	uint8_t res;
	i2c_read(self, reg, (uint8_t*)&res, 1);
	return res;
}

static uint16_t i2c_read16bit(VL53L0X_t* self, uint8_t reg)
{
	uint16_t res;
	i2c_read(self, reg, (uint8_t*)&res, 2);
	return res;
}

static uint32_t i2c_read32bit(VL53L0X_t* self, uint8_t reg)
{
	uint32_t res;
	i2c_read(self, reg, (uint8_t*)&res, 4);
	return res;
}

int VL53L0X_init(VL53L0X_t* self, uint8_t deviceAddress)
{
	memset(self, 0, sizeof(*self));
	self->i2c_receive = _i2c_empty;
	self->i2c_transmit = _i2c_empty;
	self->sleep_us = _sleep_default;
	self->deviceAddress = deviceAddress;
	return 0;
}

/* Other private functions */
static uint8_t VL53L0X_GetSpadInfo(VL53L0X_t* self, uint8_t* count, uint8_t* type_is_aperture);
static uint8_t VL53L0X_PerformSingleRefCalibration(VL53L0X_t* self, uint8_t vhv_init_byte);
static uint32_t VL53L0X_TimeoutMclksToMicroseconds(VL53L0X_t* self, uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t VL53L0X_TimeoutMicrosecondsToMclks(VL53L0X_t* self, uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static uint16_t VL53L0X_DecodeTimeout(VL53L0X_t* self, uint16_t reg_val);
static uint16_t VL53L0X_EncodeTimeout(VL53L0X_t* self, uint16_t timeout_mclks);
static uint8_t VL53L0X_GetVcselPulsePeriod(VL53L0X_t* self, vcselPeriodType type);
static void VL53L0X_GetSequenceStepEnables(VL53L0X_t* self, SequenceStepEnables * enables);
static void VL53L0X_GetSequenceStepTimeouts(VL53L0X_t* self, SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

/* API */
void VL53L0X_reg_callbacks(
		VL53L0X_t* self,
		void (*receive_cb) (uint8_t DevAddr, uint8_t* pData, uint16_t Size),
		void (*transmit_cb) (uint8_t DevAddr, uint8_t* pData, uint16_t Size),
		void (*sleep_us_cb) (uint32_t us)
		)
{
	self->i2c_receive = receive_cb;
	self->i2c_transmit = transmit_cb;
	self->sleep_us = sleep_us_cb;
}

void VL53L0X_setup(VL53L0X_t* self, float signalRateLimitMcps)
{
	// VL53L0X_DataInit() begin
	memset(self->msgBuffer, 0, sizeof(VL53L0X_MSG_BUFFER_SIZE));

	// set 2.8V mode
	i2c_write8bit(self, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
			i2c_read8bit(self, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);

	// "Set I2C standard mode"
	i2c_write8bit(self, 0x88, 0x00);

	i2c_write8bit(self, 0x80, 0x01);
	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x00, 0x00);
	self->g_stopVariable = i2c_read8bit(self, 0x91);
	i2c_write8bit(self, 0x00, 0x01);
	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x80, 0x00);

	// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	i2c_write8bit(self, MSRC_CONFIG_CONTROL, i2c_read8bit(self, MSRC_CONFIG_CONTROL) | 0x12);

	// set final range signal rate limit to 0.25 MCPS (million counts per second)
	//float signalRateLimitMcps = 0.25; // from 0 to 511.99
	i2c_write16bit(self, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, signalRateLimitMcps * (1 << 7));

	i2c_write8bit(self, SYSTEM_SEQUENCE_CONFIG, 0xFF);

	// VL53L0X_DataInit() end

	// VL53L0X_StaticInit() begin
	uint8_t spad_count;
	uint8_t spad_type_is_aperture;
	if(!VL53L0X_GetSpadInfo(self, &spad_count, &spad_type_is_aperture)) { return; }

	// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	// the API, but the same data seems to be more easily readable from
	// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t ref_spad_map[6];
	i2c_read(self, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	i2c_write8bit(self, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++)
	{
		if (i < first_spad_to_enable || spads_enabled == spad_count)
		{
			// This bit is lower than the first one that should be enabled, or
			// (reference_spad_count) bits have already been enabled, so zero this bit
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		}
		else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
		{
			spads_enabled++;
		}
	}

	i2c_write(self, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	// -- VL53L0X_set_reference_spads() end

	// -- VL53L0X_load_tuning_settings() begin
	// DefaultTuningSettings from vl53l0x_tuning.h

	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x00, 0x00);

	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x09, 0x00);
	i2c_write8bit(self, 0x10, 0x00);
	i2c_write8bit(self, 0x11, 0x00);

	i2c_write8bit(self, 0x24, 0x01);
	i2c_write8bit(self, 0x25, 0xFF);
	i2c_write8bit(self, 0x75, 0x00);

	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x4E, 0x2C);
	i2c_write8bit(self, 0x48, 0x00);
	i2c_write8bit(self, 0x30, 0x20);

	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x30, 0x09);
	i2c_write8bit(self, 0x54, 0x00);
	i2c_write8bit(self, 0x31, 0x04);
	i2c_write8bit(self, 0x32, 0x03);
	i2c_write8bit(self, 0x40, 0x83);
	i2c_write8bit(self, 0x46, 0x25);
	i2c_write8bit(self, 0x60, 0x00);
	i2c_write8bit(self, 0x27, 0x00);
	i2c_write8bit(self, 0x50, 0x06);
	i2c_write8bit(self, 0x51, 0x00);
	i2c_write8bit(self, 0x52, 0x96);
	i2c_write8bit(self, 0x56, 0x08);
	i2c_write8bit(self, 0x57, 0x30);
	i2c_write8bit(self, 0x61, 0x00);
	i2c_write8bit(self, 0x62, 0x00);
	i2c_write8bit(self, 0x64, 0x00);
	i2c_write8bit(self, 0x65, 0x00);
	i2c_write8bit(self, 0x66, 0xA0);

	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x22, 0x32);
	i2c_write8bit(self, 0x47, 0x14);
	i2c_write8bit(self, 0x49, 0xFF);
	i2c_write8bit(self, 0x4A, 0x00);

	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x7A, 0x0A);
	i2c_write8bit(self, 0x7B, 0x00);
	i2c_write8bit(self, 0x78, 0x21);

	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x23, 0x34);
	i2c_write8bit(self, 0x42, 0x00);
	i2c_write8bit(self, 0x44, 0xFF);
	i2c_write8bit(self, 0x45, 0x26);
	i2c_write8bit(self, 0x46, 0x05);
	i2c_write8bit(self, 0x40, 0x40);
	i2c_write8bit(self, 0x0E, 0x06);
	i2c_write8bit(self, 0x20, 0x1A);
	i2c_write8bit(self, 0x43, 0x40);

	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x34, 0x03);
	i2c_write8bit(self, 0x35, 0x44);

	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x31, 0x04);
	i2c_write8bit(self, 0x4B, 0x09);
	i2c_write8bit(self, 0x4C, 0x05);
	i2c_write8bit(self, 0x4D, 0x04);

	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x44, 0x00);
	i2c_write8bit(self, 0x45, 0x20);
	i2c_write8bit(self, 0x47, 0x08);
	i2c_write8bit(self, 0x48, 0x28);
	i2c_write8bit(self, 0x67, 0x00);
	i2c_write8bit(self, 0x70, 0x04);
	i2c_write8bit(self, 0x71, 0x01);
	i2c_write8bit(self, 0x72, 0xFE);
	i2c_write8bit(self, 0x76, 0x00);
	i2c_write8bit(self, 0x77, 0x00);

	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x0D, 0x01);

	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x80, 0x01);
	i2c_write8bit(self, 0x01, 0xF8);

	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x8E, 0x01);
	i2c_write8bit(self, 0x00, 0x01);
	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x80, 0x00);

	// -- VL53L0X_load_tuning_settings() end

	// "Set interrupt config to new sample ready"
	// -- VL53L0X_SetGpioConfig() begin

	i2c_write8bit(self, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	i2c_write8bit(self, GPIO_HV_MUX_ACTIVE_HIGH, i2c_read8bit(self, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
	i2c_write8bit(self, SYSTEM_INTERRUPT_CLEAR, 0x01);

	// -- VL53L0X_SetGpioConfig() end
	self->g_measTimBudUs = VL53L0X_GetMeasurementTimingBudget(self);

	// "Disable MSRC and TCC by default"
	// MSRC = Minimum Signal Rate Check
	// TCC = Target CentreCheck
	// -- VL53L0X_SetSequenceStepEnable() begin

	i2c_write8bit(self, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// -- VL53L0X_SetSequenceStepEnable() end

	// "Recalculate timing budget"
	VL53L0X_SetMeasurementTimingBudget(self, self->g_measTimBudUs);

	// VL53L0X_StaticInit() end

	// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

	// -- VL53L0X_perform_vhv_calibration() begin
	i2c_write8bit(self, SYSTEM_SEQUENCE_CONFIG, 0x01);
	if(!VL53L0X_PerformSingleRefCalibration(self, 0x40)) { return; }

	// -- VL53L0X_perform_vhv_calibration() end

	// -- VL53L0X_perform_phase_calibration() begin

	i2c_write8bit(self, SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (!VL53L0X_PerformSingleRefCalibration(self, 0x00)) { return; }

	// -- VL53L0X_perform_phase_calibration() end

	// "restore the previous Sequence Config"
	i2c_write8bit(self, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// VL53L0X_PerformRefCalibration() end
}

void VL53L0X_start_continuous(VL53L0X_t* self, uint32_t period_ms)
{
	i2c_write8bit(self, 0x80, 0x01);
	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x00, 0x00);
	i2c_write8bit(self, 0x91, self->g_stopVariable);
	i2c_write8bit(self, 0x00, 0x01);
	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x80, 0x00);

	if (period_ms != 0)
	{
		// continuous timed mode
		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
		uint16_t osc_calibrate_val = i2c_read16bit(self, OSC_CALIBRATE_VAL);

		if (osc_calibrate_val != 0)
		{
			period_ms *= osc_calibrate_val;
		}

		i2c_write32bit(self, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
		i2c_write8bit(self, SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
	}
	else
	{
		// continuous back-to-back mode
		i2c_write8bit(self, SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}
}

void VL53L0X_stop_continuous(VL53L0X_t* self)
{
	i2c_write8bit(self, SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x00, 0x00);
	i2c_write8bit(self, 0x91, 0x00);
	i2c_write8bit(self, 0x00, 0x01);
	i2c_write8bit(self, 0xFF, 0x00);
}

uint16_t VL53L0X_read_continuous_mm(VL53L0X_t* self)
{
	uint8_t tempBuffer[12];
	uint16_t temp;

	int timeoutCnt = timeoutUsMax;
	while ((i2c_read8bit(self, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
		if (!(timeoutCnt--))
		{
				self->g_isTimeout = 1;
				return 65535;
		}
		self->sleep_us(1);
	}

	// Register map starting at 0x14
	//     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	//    5A 06 BC 04 00 85 00 38 00 19 06 B6 00 00 00 00
	//   0: Ranging status, uint8_t
	//   1: ???
	// 3,2: Effective SPAD return count, uint16_t, fixpoint8.8
	//   4: 0 ?
	//   5: ???
	// 6,7: signal count rate [mcps], uint16_t, fixpoint9.7
	// 9,8: AmbientRateRtnMegaCps  [mcps], uint16_t, fixpoimt9.7
	// A,B: uncorrected distance [mm], uint16_t
	i2c_read(self, 0x14, tempBuffer, 12);
	self->rangeStatus =  tempBuffer[0x00]>>3;
	self->spadCnt     = (tempBuffer[0x02]<<8) | tempBuffer[0x03];
	self->signalCnt   = (tempBuffer[0x06]<<8) | tempBuffer[0x07];
	self->ambientCnt  = (tempBuffer[0x08]<<8) | tempBuffer[0x09];
	temp = (tempBuffer[0x0A]<<8) | tempBuffer[0x0B];
	self->rawDistance = temp;
	i2c_write8bit(self, SYSTEM_INTERRUPT_CLEAR, 0x01);
	return temp;
}

uint16_t VL53L0X_read_single_mm(VL53L0X_t* self)
{
	i2c_write8bit(self, 0x80, 0x01);
	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x00, 0x00);
	i2c_write8bit(self, 0x91, self->g_stopVariable);
	i2c_write8bit(self, 0x00, 0x01);
	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x80, 0x00);
	i2c_write8bit(self, SYSRANGE_START, 0x01);
	// "Wait until start bit has been cleared"
	int timeoutCnt = timeoutUsMax;
	while (i2c_read8bit(self, SYSRANGE_START) & 0x01){
		if (!(timeoutCnt--))
		{
			  self->g_isTimeout = 1;
			  return 65535;
		}
		self->sleep_us(1);
	}
	  return VL53L0X_read_continuous_mm(self);
}

static uint8_t VL53L0X_GetSpadInfo(VL53L0X_t* self, uint8_t* count, uint8_t* type_is_aperture)
{
	uint8_t tmp;

	i2c_write8bit(self, 0x80, 0x01);
	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x00, 0x00);

	i2c_write8bit(self, 0xFF, 0x06);
	i2c_write8bit(self, 0x83, i2c_read8bit(self, 0x83) | 0x04);
	i2c_write8bit(self, 0xFF, 0x07);
	i2c_write8bit(self, 0x81, 0x01);

	i2c_write8bit(self, 0x80, 0x01);

	i2c_write8bit(self, 0x94, 0x6b);
	i2c_write8bit(self, 0x83, 0x00);

	int timeoutCnt = timeoutUsMax;
	while (i2c_read8bit(self, 0x83) == 0x00)
	{
		if(!(timeoutCnt--)) { return 0; }
		self->sleep_us(1);
	}

	i2c_write8bit(self, 0x83, 0x01);
	tmp = i2c_read8bit(self, 0x92);

	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;

	i2c_write8bit(self, 0x81, 0x00);
	i2c_write8bit(self, 0xFF, 0x06);
	i2c_write8bit(self, 0x83, i2c_read8bit(self, 0x83)  & ~0x04);
	i2c_write8bit(self, 0xFF, 0x01);
	i2c_write8bit(self, 0x00, 0x01);

	i2c_write8bit(self, 0xFF, 0x00);
	i2c_write8bit(self, 0x80, 0x00);

	return 1;
}

uint32_t VL53L0X_GetMeasurementTimingBudget(VL53L0X_t* self)
{
	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
	uint16_t const EndOverhead        = 960;
	uint16_t const MsrcOverhead       = 660;
	uint16_t const TccOverhead        = 590;
	uint16_t const DssOverhead        = 690;
	uint16_t const PreRangeOverhead   = 660;
	uint16_t const FinalRangeOverhead = 550;

	// "Start and end overhead times always present"
	uint32_t budget_us = StartOverhead + EndOverhead;

	VL53L0X_GetSequenceStepEnables(self, &enables);
	VL53L0X_GetSequenceStepTimeouts(self, &enables, &timeouts);

	if (enables.tcc)
	{
		budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss)
	{
		budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if (enables.msrc)
	{
		budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range)
	{
		budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range)
	{
		budget_us += (timeouts.final_range_us + FinalRangeOverhead);
	}

	self->g_measTimBudUs = budget_us; // store for internal reuse
	return budget_us;

	return 0;
}

uint8_t VL53L0X_SetMeasurementTimingBudget(VL53L0X_t* self, uint32_t budget_us)
{
	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
	uint16_t const EndOverhead        = 960;
	uint16_t const MsrcOverhead       = 660;
	uint16_t const TccOverhead        = 590;
	uint16_t const DssOverhead        = 690;
	uint16_t const PreRangeOverhead   = 660;
	uint16_t const FinalRangeOverhead = 550;

	uint32_t const MinTimingBudget = 20000;

	if (budget_us < MinTimingBudget) { return 0; }

	uint32_t used_budget_us = StartOverhead + EndOverhead;

	VL53L0X_GetSequenceStepEnables(self, &enables);
	VL53L0X_GetSequenceStepTimeouts(self, &enables, &timeouts);

	if (enables.tcc)
	{
		used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss)
	{
		used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if (enables.msrc)
	{
		used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range)
	{
		used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range)
	{
		used_budget_us += FinalRangeOverhead;

		// "Note that the final range timeout is determined by the timing
		// budget and the sum of all other timeouts within the sequence.
		// If there is no room for the final range timeout, then an error
		// will be set. Otherwise the remaining time will be applied to
		// the final range."

		if (used_budget_us > budget_us)
		{
			// "Requested timeout too big."
			return 0;
		}

		uint32_t final_range_timeout_us = budget_us - used_budget_us;
		//uint32_t final_range_timeout_us = budget_us;

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."

		uint16_t final_range_timeout_mclks =
		VL53L0X_TimeoutMicrosecondsToMclks(self, final_range_timeout_us,
								 timeouts.final_range_vcsel_period_pclks);

		if (enables.pre_range)
		{
			final_range_timeout_mclks += timeouts.pre_range_mclks;
		}

		i2c_write16bit(self, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
			VL53L0X_EncodeTimeout(self, final_range_timeout_mclks));

		// set_sequence_step_timeout() end

		self->g_measTimBudUs = budget_us; // store for internal reuse
	}

	return 1;
}

// Pre (type = 0): 12 to 18 (initialized to 14 by default)
// Final(type = 1): 8 to 14 (initialized to 10 by default)
uint8_t VL53L0X_SetVcselPulsePeriod(VL53L0X_t* self, uint8_t type, uint8_t period_pclks)
{
	  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

	  SequenceStepEnables enables;
	  SequenceStepTimeouts timeouts;

		VL53L0X_GetSequenceStepEnables(self, &enables);
		VL53L0X_GetSequenceStepTimeouts(self, &enables, &timeouts);

	  // "Apply specific settings for the requested clock period"
	  // "Re-calculate and apply timeouts, in macro periods"

	  // "When the VCSEL period for the pre or final range is changed,
	  // the corresponding timeout must be read from the device using
	  // the current VCSEL period, then the new VCSEL period can be
	  // applied. The timeout then must be written back to the device
	  // using the new VCSEL period.
	  //
	  // For the MSRC timeout, the same applies - this timeout being
	  // dependant on the pre-range vcsel period."


	  if (type == VcselPeriodPreRange)
	  {
	    // "Set phase check limits"
	    switch (period_pclks)
	    {
	      case 12:
	    	  i2c_write8bit(self, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
	        break;

	      case 14:
	    	  i2c_write8bit(self, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
	        break;

	      case 16:
	    	  i2c_write8bit(self, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
	        break;

	      case 18:
	    	  i2c_write8bit(self, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
	        break;

	      default:
	        // invalid period
	        return 0;
	    }
	    i2c_write8bit(self, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

	    // apply new VCSEL period
	    i2c_write8bit(self, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

	    // update timeouts

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

	    uint16_t new_pre_range_timeout_mclks =
	      VL53L0X_TimeoutMicrosecondsToMclks(self, timeouts.pre_range_us, period_pclks);

	    i2c_write16bit(self, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
	      VL53L0X_EncodeTimeout(self, new_pre_range_timeout_mclks));

	    // set_sequence_step_timeout() end

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

	    uint16_t new_msrc_timeout_mclks =
	    		 VL53L0X_TimeoutMicrosecondsToMclks(self, timeouts.msrc_dss_tcc_us, period_pclks);

	    i2c_write8bit(self, MSRC_CONFIG_TIMEOUT_MACROP,
	      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

	    // set_sequence_step_timeout() end
	  }
	  else if (type == VcselPeriodFinalRange)
	  {
	    switch (period_pclks)
	    {
	      case 8:
	    	  i2c_write8bit(self, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
	    	  i2c_write8bit(self, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	    	  i2c_write8bit(self, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
	    	  i2c_write8bit(self, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
	    	  i2c_write8bit(self, 0xFF, 0x01);
	    	  i2c_write8bit(self, ALGO_PHASECAL_LIM, 0x30);
	    	  i2c_write8bit(self, 0xFF, 0x00);
	        break;

	      case 10:
	    	  i2c_write8bit(self, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
	    	  i2c_write8bit(self, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	    	  i2c_write8bit(self, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	    	  i2c_write8bit(self, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
	    	  i2c_write8bit(self, 0xFF, 0x01);
	    	  i2c_write8bit(self, ALGO_PHASECAL_LIM, 0x20);
	    	  i2c_write8bit(self, 0xFF, 0x00);
	        break;

	      case 12:
	    	  i2c_write8bit(self, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
	    	  i2c_write8bit(self, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	    	  i2c_write8bit(self, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	    	  i2c_write8bit(self, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
	    	  i2c_write8bit(self, 0xFF, 0x01);
	    	  i2c_write8bit(self, ALGO_PHASECAL_LIM, 0x20);
	    	  i2c_write8bit(self, 0xFF, 0x00);
	        break;

	      case 14:
	    	  i2c_write8bit(self, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
	    	  i2c_write8bit(self, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	    	  i2c_write8bit(self, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	    	  i2c_write8bit(self, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
	    	  i2c_write8bit(self, 0xFF, 0x01);
	    	  i2c_write8bit(self, ALGO_PHASECAL_LIM, 0x20);
	    	  i2c_write8bit(self, 0xFF, 0x00);
	        break;

	      default:
	        // invalid period
	        return 0;
	    }

	    // apply new VCSEL period
	    i2c_write8bit(self, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

	    // update timeouts

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

	    // "For the final range timeout, the pre-range timeout
	    //  must be added. To do this both final and pre-range
	    //  timeouts must be expressed in macro periods MClks
	    //  because they have different vcsel periods."

	    uint16_t new_final_range_timeout_mclks =
	    		VL53L0X_TimeoutMicrosecondsToMclks(self, timeouts.final_range_us, period_pclks);

	    if (enables.pre_range)
	    {
	      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
	    }

	    i2c_write16bit(self, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
	    		VL53L0X_EncodeTimeout(self, new_final_range_timeout_mclks));

	    // set_sequence_step_timeout end
	  }
	  else
	  {
	    // invalid type
	    return 0;
	  }

	  // "Finally, the timing budget must be re-applied"

	  VL53L0X_SetMeasurementTimingBudget(self, self->g_measTimBudUs);

	  // "Perform the phase calibration. This is needed after changing on vcsel period."
	  // VL53L0X_perform_phase_calibration() begin

	  uint8_t sequence_config = i2c_read8bit(self, SYSTEM_SEQUENCE_CONFIG);
	  i2c_write8bit(self, SYSTEM_SEQUENCE_CONFIG, 0x02);

	  VL53L0X_PerformSingleRefCalibration(self, 0x0);

	  i2c_write8bit(self, SYSTEM_SEQUENCE_CONFIG, sequence_config);

	  // VL53L0X_perform_phase_calibration() end

	  return 1;
}

static uint8_t VL53L0X_PerformSingleRefCalibration(VL53L0X_t* self, uint8_t vhv_init_byte)
{
	i2c_write8bit(self, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

	int timeoutCnt = timeoutUsMax;
	while ((i2c_read8bit(self, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
	{
		if (!(timeoutCnt--)) { return 0; }
		self->sleep_us(1);
	}

	i2c_write8bit(self, SYSTEM_INTERRUPT_CLEAR, 0x01);
	i2c_write8bit(self, SYSRANGE_START, 0x00);

	return 1;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static uint32_t VL53L0X_TimeoutMclksToMicroseconds(VL53L0X_t* self, uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t VL53L0X_TimeoutMicrosecondsToMclks(VL53L0X_t* self, uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static uint16_t VL53L0X_DecodeTimeout(VL53L0X_t* self, uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
static uint16_t VL53L0X_EncodeTimeout(VL53L0X_t* self, uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

static uint8_t VL53L0X_GetVcselPulsePeriod(VL53L0X_t* self, vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(i2c_read8bit(self, PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(i2c_read8bit(self, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

static void VL53L0X_GetSequenceStepEnables(VL53L0X_t* self, SequenceStepEnables * enables)
{
	  uint8_t sequence_config = i2c_read8bit(self, SYSTEM_SEQUENCE_CONFIG);

	  enables->tcc          = (sequence_config >> 4) & 0x1;
	  enables->dss          = (sequence_config >> 3) & 0x1;
	  enables->msrc         = (sequence_config >> 2) & 0x1;
	  enables->pre_range    = (sequence_config >> 6) & 0x1;
	  enables->final_range  = (sequence_config >> 7) & 0x1;
}

static void VL53L0X_GetSequenceStepTimeouts(VL53L0X_t* self, SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
	timeouts->pre_range_vcsel_period_pclks = VL53L0X_GetVcselPulsePeriod(self, VcselPeriodPreRange);

	timeouts->msrc_dss_tcc_mclks = i2c_read8bit(self, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
	timeouts->msrc_dss_tcc_us =
			VL53L0X_TimeoutMclksToMicroseconds(self,
					  timeouts->msrc_dss_tcc_mclks,
					  timeouts->pre_range_vcsel_period_pclks);

	  timeouts->pre_range_mclks =
			  VL53L0X_DecodeTimeout(self, i2c_read16bit(self, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	  timeouts->pre_range_us =
			  VL53L0X_TimeoutMclksToMicroseconds(self,
					  timeouts->pre_range_mclks,
					  timeouts->pre_range_vcsel_period_pclks);

	  timeouts->final_range_vcsel_period_pclks = VL53L0X_GetVcselPulsePeriod(self, VcselPeriodFinalRange);

	  timeouts->final_range_mclks =
			  VL53L0X_DecodeTimeout(self, i2c_read16bit(self, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

	  if (enables->pre_range)
	  {
	    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
	  }

	  timeouts->final_range_us =
			  VL53L0X_TimeoutMclksToMicroseconds(self,
					  timeouts->final_range_mclks,
					  timeouts->final_range_vcsel_period_pclks);
}


