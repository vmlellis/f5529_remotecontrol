/*****************************************************************************
 * hmc5883l.c
 *
 * Codigo para o magnometro HMC5883L
 *
 *  Criado em: 28/02/2015
 *      Autor: Victor
 *
 *	Referencia:
 *	- Calibração:
 *	  -> https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration/blob/master/Core/Compass_header_example_ver_0_2/compass.cpp
 *	  -> http://hobbylogs.me.pn/?p=17
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *****************************************************************************/

#include "hmc5883l.h"
#include <stdlib.h>
#include "../twi_master.h"
#include "../../setup.h"

#define compass_XY_excitation 1160 // The magnetic field excitation in X and Y direction during Self Test (Calibration)
#define compass_Z_excitation 1080  // The magnetic field excitation in Z direction during Self Test (Calibration)

static int compass_x, compass_y, compass_z;
static float compass_gain_fact = 1;
static float compass_x_gainError=1,compass_y_gainError=1,compass_z_gainError=1;
static float compass_x_offset=0, compass_y_offset=0, compass_z_offset=0;

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
	uint8_t data_b[] = { HMC5883L_GAIN_0_88 };
	uint8_t data_mode[] = { HMC5883L_MD_CONTINUOUS };

	twi_master_writeRegister(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, data_a, sizeof(data_a));
	twi_master_writeRegister(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, data_b, sizeof(data_b));
	twi_master_writeRegister(HMC5883L_ADDR, HMC5883L_REG_MODE, data_mode, sizeof(data_mode));

	compass_gain_fact = 0.73;
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

/**
 * Ler os dados calibrados
 */
void hmc5883l_read_scalled_data(int *mx, int *my, int *mz) {
	hmc5883l_read_data(&compass_x, &compass_y, &compass_z);

	*mx=compass_x*compass_gain_fact*compass_x_gainError+compass_x_offset;
	*my=compass_y*compass_gain_fact*compass_y_gainError+compass_y_offset;
	*mz=compass_z*compass_gain_fact*compass_z_gainError+compass_z_offset;
}

/**
 * Calibrar
 */
void hmc5883l_calibrate() {

	hmc5883_setMeasurement(HMC5883L_MS_POSITIVE);
	delay(70);
	hmc5883l_read_data(&compass_x, &compass_y, &compass_z);

	compass_x = compass_x*compass_gain_fact;
	compass_y = compass_y*compass_gain_fact;
	compass_z = compass_z*compass_gain_fact;

	// Offset = 1160 - Data_positive
	compass_x_gainError = (float) compass_XY_excitation/compass_x;
	compass_y_gainError = (float) compass_XY_excitation/compass_y;
	compass_z_gainError = (float) compass_Z_excitation/compass_z;

	hmc5883_setMeasurement(HMC5883L_MS_NEGATIVE);
	delay(70);
	hmc5883l_read_data(&compass_x, &compass_y, &compass_z);

	compass_x = compass_x*compass_gain_fact;
	compass_y = compass_y*compass_gain_fact;
	compass_z = compass_z*compass_gain_fact;

	// Taking the average of the offsets
	compass_x_gainError = (float)((compass_XY_excitation/abs(compass_x))+compass_x_gainError)/2;
	compass_y_gainError = (float)((compass_XY_excitation/abs(compass_y))+compass_y_gainError)/2;
	compass_z_gainError = (float)((compass_Z_excitation/abs(compass_z))+compass_z_gainError)/2;

	hmc5883_setMeasurement(HMC5883L_MS_NORMAL);
	delay(70);
}
