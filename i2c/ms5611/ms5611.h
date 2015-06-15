/*
 * ms5611.h
 *
 *  Created on: 14/04/2015
 *      Author: Victor
 */

#ifndef I2C_MS5611_MS5611_H_
#define I2C_MS5611_MS5611_H_

#include <inttypes.h>

#define MS5611_ADDRESS                0x77 // Endereço I2C

#define MS5611_CMD_ADC_READ           0x00 // Leitura do ADC (Conversor analogico digital)
#define MS5611_CMD_RESET              0x1E // RESET
#define MS5611_CMD_CONV_D1            0x40 // Convert D1
#define MS5611_CMD_CONV_D2            0x50 // Convert D2
#define MS5611_CMD_READ_PROM          0xA0 // Leitura do PROM

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;

uint8_t ms5611_detect(void);				// Verificar a presença do MS5611
void ms5611_config(ms5611_osr_t);			// Calibração do MS5611
void ms5611_read(int32_t*, int32_t*); 		// Calcula a temperatura e a pressão
void ms5611_readRealTemperature(float*); 	// Realiza a leitura da temperatura real
void ms5611_readRealPressure(float*);		// Realiza a leitura da pressão real
float ms5611_getAltitude(float);			// Calcula a altitude
void ms5611_readAltitude(float*);			// Realiza a leitura da pressão e retorna a altitude


#endif /* I2C_MS5611_MS5611_H_ */

