/*****************************************************************************
 * hmc5883l.c
 *
 * Codigo para o magnometro HMC5883L
 *
 *  Criado em: 28/02/2015
 *      Autor: Victor
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *****************************************************************************/

#include "hmc5883l.h"
#include "../twi_master.h"

/*
 * Detectar se estah no barramento
 */
uint8_t hmc5883l_detect(void) {
	uint8_t hmlc5883_RxBuffer[3] = {'0', '0', '0'};
	twi_master_readRegister(HMC5883L_ADDR, HMC5883L_REG_ID_A, hmlc5883_RxBuffer, sizeof(hmlc5883_RxBuffer));

	return (hmlc5883_RxBuffer[0] == 'H' && hmlc5883_RxBuffer[1] == '4' && hmlc5883_RxBuffer[2] == '3');
}

/*
 * Configuração
 */
void hmc5883l_config(void) {
	uint8_t data_a[] = { HMC5883L_AVG_8 | HMC5883L_HZ_15 | HMC5883L_MS_NORMAL };
	uint8_t data_b[] = { HMC5883L_GAIN_1_33 };
	uint8_t data_mode[] = { HMC5883L_MD_CONTINUOUS };

	twi_master_writeRegister(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, data_a, sizeof(data_a));
	twi_master_writeRegister(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, data_b, sizeof(data_b));
	twi_master_writeRegister(HMC5883L_ADDR, HMC5883L_REG_MODE, data_mode, sizeof(data_mode));
}

/*
 * Setar o tipo de leitura
 */
void hmc5883_setMeasurement(uint8_t data) {
	twi_master_writeBits(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 1, 2, data);
}

/*
 * Ler os dados
 */
void hmc5883l_read_data(int *mx, int *my, int *mz) {
	uint8_t hmlc5883_RxBuffer[6];
	twi_master_readRegister(HMC5883L_ADDR, HMC5883L_REG_X_MSB, hmlc5883_RxBuffer, sizeof(hmlc5883_RxBuffer));
	*mx = hmlc5883_RxBuffer[1] | hmlc5883_RxBuffer[0] <<8;
	*mz = hmlc5883_RxBuffer[3] | hmlc5883_RxBuffer[2] <<8;
	*my = hmlc5883_RxBuffer[5] | hmlc5883_RxBuffer[4] <<8;

}
