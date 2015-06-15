/*****************************************************************************
 * hmc5883l.h
 *
 * Biblioteca para o magnometro HMC5883L
 *
 *  Criado em: 28/02/2015
 *      Autor: Victor
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *****************************************************************************/

#ifndef I2C_HMC5883L_HMC5883L_H_
#define I2C_HMC5883L_HMC5883L_H_

#include <inttypes.h>

// ADDRESS
#define HMC5883L_ADDR    0x1e

// REGS
#define HMC5883L_REG_CONFIG_A 0
#define HMC5883L_REG_CONFIG_B 1
#define HMC5883L_REG_MODE     2
#define HMC5883L_REG_X_MSB    3
#define HMC5883L_REG_X_LSB    4
#define HMC5883L_REG_Z_MSB    5
#define HMC5883L_REG_Z_LSB    6
#define HMC5883L_REG_Y_MSB    7
#define HMC5883L_REG_Y_LSB    8
#define HMC5883L_REG_STATUS   9
#define HMC5883L_REG_ID_A     10
#define HMC5883L_REG_ID_B     11
#define HMC5883L_REG_ID_C     12

///////////////////////////////////////
// REG CONFIG A
#define HMC5883L_AVG_1 1<<5
#define HMC5883L_AVG_2 2<<5
#define HMC5883L_AVG_4 3<<5
#define HMC5883L_AVG_8 4<<5

#define HMC5883L_HZ_0_75 0<<2
#define HMC5883L_HZ_1_5  1<<2
#define HMC5883L_HZ_3    2<<2
#define HMC5883L_HZ_7_5  3<<2
#define HMC5883L_HZ_15   4<<2
#define HMC5883L_HZ_30   5<<2
#define HMC5883L_HZ_75   6<<2

#define HMC5883L_MS_NORMAL   0
#define HMC5883L_MS_POSITIVE 1
#define HMC5883L_MS_NEGATIVE 2

///////////////////////////////////////
// REG CONFIG B
#define HMC5883L_GAIN_0_88 0<<5
#define HMC5883L_GAIN_1_33 1<<5
#define HMC5883L_GAIN_1_9  2<<5
#define HMC5883L_GAIN_2_5  3<<5
#define HMC5883L_GAIN_4    4<<5
#define HMC5883L_GAIN_4_7  5<<5
#define HMC5883L_GAIN_5_6  6<<5
#define HMC5883L_GAIN_8_1  7<<5

///////////////////////////////////////
// REG CONFIG MODE
#define HMC5883L_MD_CONTINUOUS 0
#define HMC5883L_MD_SINGLE     1

uint8_t hmc5883l_detect(void);
void hmc5883l_config(void);
void hmc5883_setMeasurement(uint8_t);
void hmc5883l_read_data(int*, int*, int*);

#endif /* I2C_HMC5883L_HMC5883L_H_ */
