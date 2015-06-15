/*
 * ms5611.c
 *
 *  Created on: 14/04/2015
 *      Author: Victor
 */

#include <math.h>
#include "ms5611.h"
#include "../twi_master.h"

//extern void uart_printf(char *format, ...);

extern void delay(uint32_t milliseconds);

static uint16_t fc[8];
static uint8_t ct;
static uint8_t uosr;

// Pressão no nivel do mar (em Pa)
const uint32_t p0 = 1013.25;

/*
 * Detecta se o dispositivo estah no barramento
 */
uint8_t ms5611_detect(void) {
	uint8_t data = 0;
	uint8_t bytesRead = twi_master_readRegister(MS5611_ADDRESS, MS5611_CMD_READ_PROM, &data, sizeof(data));
	return (bytesRead > 0);
}

/*
 * Realiza o reset
 */
void ms5611_reset(void) {
	uint8_t cmd = MS5611_CMD_RESET;
	twi_master_writeTo(MS5611_ADDRESS, &cmd, sizeof(cmd), 1);
}

/*
 * Leitura do PROM
 */
static void ms5611_readPROM(void) {
	uint8_t offset = 0;
	for (offset = 0; offset < 8; offset++)
	{
		fc[offset] = twi_master_read16(MS5611_ADDRESS, MS5611_CMD_READ_PROM + (offset * 2));
	}
}

/*
 * Setar Oversampling
 */
static void ms5611_setOversampling(ms5611_osr_t osr) {
	switch (osr) {
		case MS5611_ULTRA_LOW_POWER:
		    ct = 1;
		    break;
		case MS5611_LOW_POWER:
		    ct = 2;
		    break;
		case MS5611_STANDARD:
		    ct = 3;
		    break;
		case MS5611_HIGH_RES:
		    ct = 5;
		    break;
		case MS5611_ULTRA_HIGH_RES:
		    ct = 10;
		    break;
	}

	uosr = osr;
}

/*
 * Configuração inicial
 */
void ms5611_config(ms5611_osr_t osr) {
	ms5611_reset();
	ms5611_setOversampling(osr);
	delay(100);
	ms5611_readPROM();
}

/*
 * Leitura de temperatura raw
 */
static void ms5611_readRawPressure(uint32_t* rawPressure) {
	uint8_t data = MS5611_CMD_CONV_D1 + uosr;
	twi_master_writeTo(MS5611_ADDRESS, &data, sizeof(data), 1);
	delay(ct);
	*rawPressure = twi_master_read24(MS5611_ADDRESS, MS5611_CMD_ADC_READ);
}

/*
 *Leitura de pressão raw
 */
static void ms5611_readRawTemperature(uint32_t* rawTemperature) {
	uint8_t data = MS5611_CMD_CONV_D2 + uosr;
	twi_master_writeTo(MS5611_ADDRESS, &data, sizeof(data), 1);
	delay(ct);
	*rawTemperature = twi_master_read24(MS5611_ADDRESS, MS5611_CMD_ADC_READ);
}

/*
 * Leitura da temperatura e pressão
 */
void ms5611_read(int32_t* temperature, int32_t* pressure) {
	uint32_t press;
	int64_t temp;
	int64_t delt;
	uint32_t d1, d2;

	ms5611_readRawPressure(&d1);
	ms5611_readRawTemperature(&d2);

	int32_t dT = d2 - ((uint32_t)fc[5] << 8);
	int64_t off = ((int64_t)fc[2] << 16) + (((int64_t)fc[4] * dT) >> 7);
	int64_t sens = ((int64_t)fc[1] << 15) + (((int64_t)fc[3] * dT) >> 8);

	temp = 2000 + ((dT * (int64_t)fc[6]) >> 23);

	if (temp < 2000) { // temperature lower than 20degC
		delt = temp - 2000;
		delt = 5 * delt * delt;
		off -= delt >> 1;
		sens -= delt >> 2;
		if (temp < -1500) { // temperature lower than -15degC
			delt = temp + 1500;
			delt = delt * delt;
			off -= 7 * delt;
			sens -= (11 * delt) >> 1;
		}
	}

	press = ((((int64_t)d1 * sens) >> 21) - off) >> 15;


	if (temperature)
		*temperature = temp;
	if (pressure)
		*pressure = press;
}

void ms5611_readRealTemperature(float* realTemperature) {
	int64_t temp;
	uint32_t d1, d2;

	ms5611_readRawPressure(&d1);
	ms5611_readRawTemperature(&d2);

	int32_t dT = d2 - ((uint32_t)fc[5] << 8);
	temp = 2000 + ((dT * (int64_t)fc[6]) >> 23);

	*realTemperature = temp/100.f;
}

/*
 * Pressão em mbar
 */
void ms5611_readRealPressure(float* realPressure) {
	int32_t press;
	ms5611_read(0, &press);
	*realPressure = press/100.0f;
}

/*
 * Ler a altitude com a pressão lida do MS5611
 */
float ms5611_getAltitude(float pressure) {
	return (float)44307.694f * (1.0 - powf(((float) pressure/p0), 0.1902949f));
}


/*
 * Ler a altitude em m
 */
void ms5611_readAltitude(float* altitude) {
	float pressure = 0.0;
	ms5611_readRealPressure(&pressure);
	*altitude = ms5611_getAltitude(pressure);
}
