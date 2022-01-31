/*
 * pulse_oximiter.c
 *
 *  Created on: Jan 29, 2022
 *      Author: Mario Regus
 *
 *  Note: Contains code adapted from Raivis Strogonivs
 *  https://morf.lv/implementing-pulse-oximeter-using-max30100
 *  https://github.com/xcoder123/MAX30100
 */

#include "pulse_oximeter.h"
#include "system.h"
#include "uart.h"
#include "filter.h"
#include "math.h"

I2C_HandleTypeDef hi2c1;

MAX30102 pulseOximeter = {0};
FIFO_LED_DATA fifoData = {0};

MEASUREMENT_MODE measurementMode = SPO2;
POWER_MODE powerMode = NORMAL;
SAMPLE_RATE sampleRate = _50SPS;

uint8_t fifoRdPtr = 0;

DC_FILTER_T dcFilterIR = {0};
DC_FILTER_T dcFilterRed = {0};
MEAN_DIFF_FILTER_T meanDiffIR = {0};
BUTTERWORTH_FILTER_T lpbFilterIR = {0};

float currentBPM;
float valuesBPM[PULSE_BPM_SAMPLE_SIZE] = {0};
float valuesBPMSum = 0;
uint8_t valuesBPMCount = 0;
uint8_t bpmIndex = 0;
uint32_t lastBeatThreshold = 0;

float irACValueSqSum = 0;
float redACValueSqSum = 0;
uint16_t samplesRecorded = 0;
uint16_t pulsesDetected = 0;
float currentSpO2Value = 0;

uint8_t redLEDCurrent = 0;
float lastREDLedCurrentCheck = 0;
PULSE_STATE_MACHINE currentPulseDetectorState = PULSE_IDLE;

LEDCurrent IrLedCurrent;

/**
* @brief I2C1 Initialization Function
* @param None
* @retval None
*/
void I2C_Init(void)
{
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	// Call the HAL error handler on error
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		errorHandler();
	}
}

int8_t pulseOximeter_readRegister(uint8_t reg, uint8_t* value)
{
	HAL_StatusTypeDef retStatus;
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = 0x03;

	uint8_t address = (I2C_SLAVE_ID | I2C_WRITE);

	retStatus = HAL_I2C_Master_Transmit(&hi2c1, address, buf, 1, HAL_MAX_DELAY);
	if( retStatus != HAL_OK ){
		return -1;
	}

	address = (I2C_SLAVE_ID | I2C_READ);
	retStatus = HAL_I2C_Master_Receive(&hi2c1, address, buf, 1, HAL_MAX_DELAY);
	if( retStatus != HAL_OK ){
		return -1;
	}

	*value = buf[0];

	return 0;
}

HAL_StatusTypeDef pulseOximeter_writeRegister(uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef retStatus;
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;

	uint8_t address = (I2C_SLAVE_ID | I2C_WRITE);
	retStatus = HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, HAL_MAX_DELAY);

	return retStatus;
}


void pulseOximeter_setMeasurementMode(MEASUREMENT_MODE mode)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x7 << 0);

	switch(mode){
	case HEART_RATE:	readResult = readResult | (0x02 << 0); break;
	case SPO2:	readResult = readResult | (0x03 << 0); break;
	case MULTI_LED:	readResult = readResult | (0x07 << 0); break;
	default: return; break;
	}

	if( pulseOximeter_writeRegister(MODE_CONFIG, readResult) != HAL_OK){
		return;
	}
	else{
		measurementMode = mode;
	}
}

MEASUREMENT_MODE pulseOximeter_getMeasurementMode(void)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return MEASUREMENT_MODE_FAIL;
	}

	readResult &= 0x07;

	return (MEASUREMENT_MODE)readResult;

	switch(readResult)
	{
		case 2: return HEART_RATE;	break;
		case 3: return SPO2;	break;
		case 7: return MULTI_LED;	break;
		default: return HEART_RATE;	break;
	}
}

void pulseOximeter_setPowerMode(POWER_MODE mode)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x80 << 0);

	switch(mode){
	case NORMAL:	readResult = readResult | (0x00 << 0); break;
	case LOW_POWER:	readResult = readResult | (0x80 << 0); break;
	default: return; break;
	}

	if( pulseOximeter_writeRegister(MODE_CONFIG, readResult) != HAL_OK){
		return;
	}
	else{
		measurementMode = mode;
	}
}

POWER_MODE pulseOximeter_getPowerMode(void)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return POWER_MODE_FAIL;
	}

	readResult &= 0x80;

	switch(readResult)
	{
		case 0: return NORMAL;	break;
		case 0x80: return LOW_POWER;	break;
		default: return NORMAL; break;
	}
}

void pulseOximeter_resetRegisters(void)
{
	int8_t readStatus;
	uint8_t readResult;


	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x01 << 6);
	readResult = readResult | (0x01 << 6);
	if( pulseOximeter_writeRegister(MODE_CONFIG, readResult) != HAL_OK){
		return;
	}
}

void pulseOximeter_setLedCurrent(uint8_t led, float currentLevel)
{
	uint8_t value = 0;
	uint8_t ledRegister = 0;

	switch(led){
	case RED_LED: ledRegister = LED_PULSE_AMP_1; break;
	case IR_LED:	ledRegister = LED_PULSE_AMP_2; break;
	}

	// slope derived from MAX30102 DataSheet
	value = (uint8_t)(5.0 * currentLevel);

	if( pulseOximeter_writeRegister(ledRegister, value) != HAL_OK){
		return;
	}
	else{

	}
}

float pulseOximeter_getLedCurrent(uint8_t led)
{
	int8_t readStatus = 0;
	float currentLevel;
	uint8_t ledRegister = 0;
	uint8_t readResult;

	switch(led){
	case RED_LED: ledRegister = LED_PULSE_AMP_1; break;
	case IR_LED:	ledRegister = LED_PULSE_AMP_2; break;
	}

	readStatus = pulseOximeter_readRegister(ledRegister, &readResult);
	if( readStatus == -1){
		return -1;
	}

	currentLevel = readResult / 5.0;

	return currentLevel;
}

void pulseOximeter_setSampleRate(uint8_t sampleRate)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x1C << 0);

	readResult = readResult | (sampleRate << 2);

	if( pulseOximeter_writeRegister(SPO2_CONFIG, readResult) != HAL_OK){
		return;
	}
	else{

	}
}

SAMPLE_RATE pulseOximeter_getSampleRate(void)
{
	int8_t readStatus = 0;
	uint8_t result;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		return _SAMPLE_RATE_FAIL;
	}

	result = readResult;
	result &= 0x1C;
	result = result >> 2;

	return (SAMPLE_RATE)result;
}

void pulseOximeter_setPulseWidth(uint8_t pulseWidth)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x03 << 0);

	switch(pulseWidth)
	{
	case _69_US: readResult = readResult | 0; break;
	case _118_US: readResult = readResult | (0x01 << 0); break;
	case _215_US: readResult = readResult | (0x02 << 0); break;
	case _411_US: readResult = readResult | (0x03 << 0); break;
	case _PULSE_WIDTH_FAIL: break;
	}

	if( pulseOximeter_writeRegister(SPO2_CONFIG, readResult) != HAL_OK){
		return;
	}
	else{

	}
}

PULSE_WIDTH pulseOximeter_getPulseWidth(void)
{
	int8_t readStatus = 0;
		uint8_t result;
		uint8_t readResult;

		readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
		if( readStatus == -1){
			return _PULSE_WIDTH_FAIL;
		}

		result = readResult;
		result &= 0x03;

		return (PULSE_WIDTH)result;
}

// Write zero to all FIFO registers
void pulseOximeter_resetFifo(void)
{
	pulseOximeter_writeRegister(FIFO_WRITE_PTR, 0);
	pulseOximeter_writeRegister(FIFO_READ_POINTER, 0);
	pulseOximeter_writeRegister(FIFO_OVF_COUNTER, 0);
}

//
void pulseOximeter_initFifo(void)
{
	// Check if polling/interrupt mode
	if( PUSLE_OXIMETER_INTERRUPT == 1 )
	{
		pulseOximeter_writeRegister(FIFO_CONFIG, 0x0F);

		// FIFO almost full interrupt enable
		pulseOximeter_writeRegister(INT_ENABLE_1, 0x40);

		pulseOximeter_clearInterrupt();
	}else{
		pulseOximeter_writeRegister(FIFO_CONFIG, 0x0F);
		pulseOximeter_writeRegister(INT_ENABLE_1, 0x00);
	}
}

FIFO_LED_DATA pulseOximeter_readFifo(void)
{
	uint8_t address;uint8_t buf[12];
	uint8_t numBytes = 6;

	buf[0] = FIFO_DATA;

	address = (I2C_SLAVE_ID | I2C_WRITE);

	HAL_I2C_Master_Transmit(&hi2c1, address, buf, 1, HAL_MAX_DELAY);

	address = (I2C_SLAVE_ID | I2C_READ);
	HAL_I2C_Master_Receive(&hi2c1, address, buf, numBytes, HAL_MAX_DELAY);

	fifoData.irLedRaw = 0;
	fifoData.redLedRaw = 0;

	fifoData.irLedRaw = (buf[4] << 8) | (buf[5] << 0);
	fifoData.redLedRaw =(buf[1] << 8) | (buf[0] << 0);

	return fifoData;
}


int8_t pulseOximeter_readFIFO(uint8_t* dataBuf, uint8_t numBytes)
{
	HAL_StatusTypeDef retStatus;
	uint8_t buf[12];

	buf[0] = FIFO_DATA;

	uint8_t address = (I2C_SLAVE_ID | I2C_WRITE);

	retStatus = HAL_I2C_Master_Transmit(&hi2c1, address, buf, 1, HAL_MAX_DELAY);
	if( retStatus != HAL_OK ){
		return -1;
	}

	address = (I2C_SLAVE_ID | I2C_READ);
	retStatus = HAL_I2C_Master_Receive(&hi2c1, address, buf, numBytes, HAL_MAX_DELAY);
	if( retStatus != HAL_OK ){
		return -1;
	}

	for(int i = 0; i < numBytes; i++)
	{
		*dataBuf = buf[i];
		dataBuf++;
	}

	return 0;
}

// Read the INT STATUS register to clear interrupt
void pulseOximeter_clearInterrupt(void)
{
	uint8_t readResult;

	pulseOximeter_readRegister(INT_STATUS_1, &readResult);
}


float pulseOximeter_readTemperature(void)
{
	uint8_t tempDone = 1;
	uint8_t readResult;
	int8_t tempFraction;
	uint8_t tempInteger;
	float temperature;

	// Initiate a temperature conversion
	pulseOximeter_writeRegister(DIE_TEMP_CONFIG, 1);

	// Wait for conversion finish
	while( tempDone != 0 )
	{
		pulseOximeter_readRegister(DIE_TEMP_CONFIG, &tempDone);
	}

	// Read Die temperature integer register
	pulseOximeter_readRegister(DIE_TEMP_INTEGER, &readResult);
	tempInteger = readResult;

	// Read Die temperature fraction register
	pulseOximeter_readRegister(DIE_TEMP_FRACTION, &readResult);
	tempFraction = readResult;

	// Conversion factor found in MAX30102 DataSheet
	temperature = tempInteger + (tempFraction*0.0625);

	return temperature;
}

MAX30102 pulseOximeter_update(FIFO_LED_DATA m_fifoData)
{
	MAX30102 result = {
	/*bool pulseDetected*/ false,
	/*float heartBPM*/ 0.0,
	/*float irCardiogram*/ 0.0,
	/*float irDcValue*/ 0.0,
	/*float redDcValue*/ 0.0,
	/*float SaO2*/ currentSpO2Value,
	/*uint32_t lastBeatThreshold*/ 0,
	/*float dcFilteredIR*/ 0.0,
	/*float dcFilteredRed*/ 0.0,
	/*float temperature;*/ 0.0
  };

	result.temperature = pulseOximeter_readTemperature();

	dcFilterIR = dcRemoval( (float)m_fifoData.irLedRaw, dcFilterIR.w, ALPHA );
	dcFilterRed = dcRemoval( (float)m_fifoData.redLedRaw, dcFilterRed.w, ALPHA );

	float meanDiffResIR = meanDiff( dcFilterIR.result, &meanDiffIR);
	lowPassButterworthFilter( meanDiffResIR/*-dcFilterIR.result*/, &lpbFilterIR );

	irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
	redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
	samplesRecorded++;

	if( detectPulse( lpbFilterIR.result ) && samplesRecorded > 0 )
	{
		result.pulseDetected=true;
		pulsesDetected++;

		float ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );

		//This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration
		currentSpO2Value = 110.0 - 18.0 * ratioRMS;
		result.SpO2 = currentSpO2Value;

		if( pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
		{
			irACValueSqSum = 0;
			redACValueSqSum = 0;
			samplesRecorded = 0;
		}
	}

	balanceIntesities( dcFilterRed.w, dcFilterIR.w );


	result.heartBPM = currentBPM;
	result.irCardiogram = lpbFilterIR.result;
	result.irDcValue = dcFilterIR.w;
	result.redDcValue = dcFilterRed.w;
	result.lastBeatThreshold = lastBeatThreshold;
	result.dcFilteredIR = dcFilterIR.result;
	result.dcFilteredRed = dcFilterRed.result;

	return result;
}


bool detectPulse(float sensor_value)
{
  static float prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;

  if(sensor_value > PULSE_MAX_THRESHOLD)
  {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return false;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
      if(sensor_value > prev_sensor_value)
      {
        currentBeat = millis();
        lastBeatThreshold = sensor_value;
      }
      else
      {
    	uint32_t beatDuration = currentBeat - lastBeat;
        lastBeat = currentBeat;

        float rawBPM = 0;
        if(beatDuration > 0)
          rawBPM = 60000.0 / (float)beatDuration;

        valuesBPM[bpmIndex] = rawBPM;
        valuesBPMSum = 0;

        for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
        {
          valuesBPMSum += valuesBPM[i];
        }

        bpmIndex++;
        bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

        if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
          valuesBPMCount++;

        currentBPM = valuesBPMSum / valuesBPMCount;


        currentPulseDetectorState = PULSE_TRACE_DOWN;

        return true;
      }
      break;

    case PULSE_TRACE_DOWN:
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }


      if(sensor_value < PULSE_MIN_THRESHOLD)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return false;
}

void balanceIntesities( float redLedDC, float IRLedDC )
{
	uint32_t currentTime = millis();
  if( currentTime - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS)
  {
	  if( IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30100_LED_CURRENT_50MA)
    {
      redLEDCurrent++;
      pulseOximeter_setLedCurrent(RED_LED, redLEDCurrent);
	  pulseOximeter_setLedCurrent(IR_LED, IrLedCurrent);
    }
    else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0)
    {
      redLEDCurrent--;pulseOximeter_setLedCurrent(RED_LED, redLEDCurrent);
      pulseOximeter_setLedCurrent(IR_LED, IrLedCurrent);
    }

    lastREDLedCurrentCheck = millis();
  }
}

// Print registers out to uart
void pulseOximiter_registerDump(void)
{
	int8_t readStatus;
	uint8_t readResult;
	//uint8_t buf[12];

	readStatus = pulseOximeter_readRegister(INT_STATUS_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	//itoa(readResult, buf, 16);   // here 16 means Hexadecimal
	uart_PrintString("INT_STATUS_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(INT_STATUS_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	//itoa(readResult, buf, 16);   // here 16 means Hexadecimal
	uart_PrintString("INT_STATUS_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(INT_ENABLE_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	//itoa(readResult, buf, 16);   // here 16 means Hexadecimal
	uart_PrintString("INT_ENABLE_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(INT_ENABLE_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("INT_ENABLE_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_WRITE_PTR, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_WRITE_PTR: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_OVF_COUNTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_OVF_COUNTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_READ_POINTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_READ_POINTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_DATA, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_DATA_REGISTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MODE_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("SPO2_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(LED_PULSE_AMP_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("LED_PULSE_AMP_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(LED_PULSE_AMP_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("LED_PULSE_AMP_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(LED_MODE_CONTROL_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("LED_MODE_CONTROL_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(LED_MODE_CONTROL_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("LED_MODE_CONTROL_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(DIE_TEMP_INTEGER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("DIE_TEMP_INTEGER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(DIE_TEMP_FRACTION, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("DIE_TEMP_FRACTION: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(DIE_TEMP_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("DIE_TEMP_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(REVISION_ID, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("REVISION_ID: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(PART_ID, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("PART_ID: 0x");
	uart_PrintInt(readResult, 16);

	uart_PrintString("\r\n");
}

void pulseOximeter_displayData(void)
{
	uart_PrintString("MAX30102 Pulse Oximiter Data:\r\n");

	uart_PrintString("Die Temperature: ");
	uart_PrintFloat(pulseOximeter.temperature);

	uart_PrintString("Heart Rate (BPM): ");
	uart_PrintFloat(pulseOximeter.heartBPM);

	uart_PrintString("Oxygen Saturation SpO2 (%): ");
	uart_PrintFloat(pulseOximeter.SpO2);
	uart_PrintString("\r\n");
}
