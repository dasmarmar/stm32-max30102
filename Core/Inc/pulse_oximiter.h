/*
 * pulse_oximiter.h
 *
 *  Created on: Jan 29, 2022
 *      Author: Mario Regus
 *
 *  Note: Contains code adapted from Raivis Strogonivs
 *  https://morf.lv/implementing-pulse-oximeter-using-max30100
 *  https://github.com/xcoder123/MAX30100
 */

#ifndef __PULSE_OXIMITER_H__
#define __PULSE_OXIMITER_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stdbool.h"

/* Pulse detection parameters */
#define PULSE_MIN_THRESHOLD         100 //300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
#define PULSE_MAX_THRESHOLD         2000//2000
#define PULSE_GO_DOWN_THRESHOLD     1

#define PULSE_BPM_SAMPLE_SIZE       10 //Moving average size

/* SpO2 parameters */
#define RESET_SPO2_EVERY_N_PULSES     4

/* Adjust RED LED current balancing*/
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF         65000
#define RED_LED_CURRENT_ADJUSTMENT_MS           500

#define DEFAULT_SAMPLING_RATE             MAX30100_SAMPLING_RATE_100HZ
#define DEFAULT_LED_PULSE_WIDTH           MAX30100_PULSE_WIDTH_1600US_ADC_16

#define DEFAULT_IR_LED_CURRENT            MAX30100_LED_CURRENT_50MA
#define STARTING_RED_LED_CURRENT          MAX30100_LED_CURRENT_27_1MA

/// MAX30102 Register Address Map
///
/// Status
#define INT_STATUS_1			0x00
#define INT_STATUS_2			0x01
#define INT_ENABLE_1			0x02
#define INT_ENABLE_2			0x03
/// FIFO
#define FIFO_WRITE_PTR			0x04
#define FIFO_OVF_COUNTER		0x05
#define FIFO_READ_POINTER		0x06
#define FIFO_DATA				0x07
/// Configuration
#define FIFO_CONFIG				0x08
#define MODE_CONFIG				0x09
#define SPO2_CONFIG				0x0A
#define LED_PULSE_AMP_1			0x0C
#define LED_PULSE_AMP_2			0x0D
#define LED_MODE_CONTROL_1		0x11
#define LED_MODE_CONTROL_2		0x12
/// Die Temperature
#define DIE_TEMP_INTEGER		0x1F
#define DIE_TEMP_FRACTION		0x20
#define DIE_TEMP_CONFIG			0x21
/// Part ID
#define REVISION_ID				0xFE
#define PART_ID					0xFF

#define I2C_SLAVE_ID			0xAE
#define I2C_WRITE				0x00
#define I2C_READ				0x01

#define RED_LED	1
#define IR_LED	2

#define DATA_READ_FAIL			0xFFFFFFFF

typedef enum{
	HEART_RATE,
	SPO2,
	MULTI_LED,
	MEASUREMENT_MODE_FAIL
}MEASUREMENT_MODE;

typedef enum{
	NORMAL,
	LOW_POWER,
	POWER_MODE_FAIL
}POWER_MODE;

typedef enum{
	_50SPS,
	_100SPS,
	_200SPS,
	_400SPS,
	_800SPS,
	_1000SPS,
	_1600SPS,
	_3200SPS,
	_SAMPLE_RATE_FAIL
}SAMPLE_RATE;

typedef enum{
	_69_US,
	_118_US,
	_215_US,
	_411_US,
	_PULSE_WIDTH_FAIL
}PULSE_WIDTH;

extern SAMPLE_RATE sampleRate;

extern MEASUREMENT_MODE measurementMode;
extern POWER_MODE powerMode;

typedef struct{
	uint16_t redLedRaw;
	uint16_t irLedRaw;
}FIFO_LED_DATA;

typedef enum LEDCurrent {
    MAX30100_LED_CURRENT_0MA              = 0x00,
    MAX30100_LED_CURRENT_4_4MA            = 0x01,
    MAX30100_LED_CURRENT_7_6MA            = 0x02,
    MAX30100_LED_CURRENT_11MA             = 0x03,
    MAX30100_LED_CURRENT_14_2MA           = 0x04,
    MAX30100_LED_CURRENT_17_4MA           = 0x05,
    MAX30100_LED_CURRENT_20_8MA           = 0x06,
    MAX30100_LED_CURRENT_24MA             = 0x07,
    MAX30100_LED_CURRENT_27_1MA           = 0x08,
    MAX30100_LED_CURRENT_30_6MA           = 0x09,
    MAX30100_LED_CURRENT_33_8MA           = 0x0A,
    MAX30100_LED_CURRENT_37MA             = 0x0B,
    MAX30100_LED_CURRENT_40_2MA           = 0x0C,
    MAX30100_LED_CURRENT_43_6MA           = 0x0D,
    MAX30100_LED_CURRENT_46_8MA           = 0x0E,
    MAX30100_LED_CURRENT_50MA             = 0x0F
} LEDCurrent;

extern LEDCurrent IrLedCurrent;


typedef struct {
  bool pulseDetected;
  float heartBPM;
  float irCardiogram;
  float irDcValue;
  float redDcValue;
  float SpO2;
  uint32_t lastBeatThreshold;
  float dcFilteredIR;
  float dcFilteredRed;
  float temperature;
}MAX30102;

extern MAX30102 pulseOximiter;


typedef enum {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} PULSE_STATE_MACHINE;

extern PULSE_STATE_MACHINE currentPulseDetectorState;


void I2C_Init(void);

void pulseOximiter_setMeasurementMode(uint8_t mode);
MEASUREMENT_MODE pulseOximiter_getMeasurementMode(void);
void pulseOximiter_setPowerMode(POWER_MODE mode);
POWER_MODE pulseOximiter_getPowerMode(void);
int8_t pulseOximiter_readRegister(uint8_t reg, uint8_t* value);
HAL_StatusTypeDef pulseOximiter_writeRegister(uint8_t reg, uint8_t value);
void pulseOximiter_setSampleRate(uint8_t sampleRate);
SAMPLE_RATE pulseOximiter_getSampleRate(void);
void pulseOximiter_setLedCurrent(uint8_t led, float currentLevel);
float pulseOximiter_getLedCurrent(uint8_t led);
void pulseOximiter_setPulseWidth(uint8_t pulseWidth);
PULSE_WIDTH pulseOximiter_getPulseWidth(void);
int8_t pulseOximiter_readFIFO(uint8_t* dataBuf, uint8_t numBytes);
void pulseOximiter_resetRegisters(void);

void pulseOximiter_resetFifo(void);
FIFO_LED_DATA pulseOximiter_readFifo(void);
float pulseOximiter_readTemperature(void);
void pulseOximiter_initFifo(void);
void pulseOximiter_clearInterrupt(void);
MAX30102 pulseOximiter_update(FIFO_LED_DATA m_fifoData);
bool detectPulse(float sensor_value);
void balanceIntesities( float redLedDC, float IRLedDC );
void pulseOximiter_displayData(void);

#endif /* __PULSE_OXIMITER_H__ */
