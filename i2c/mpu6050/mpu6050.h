/*
 * mpu6050.h
 *
 *  Created on: 09/03/2015
 *      Author: Victor
 */

#ifndef I2C_MPU6050_MPU6050_H_
#define I2C_MPU6050_MPU6050_H_

#include <inttypes.h>

#define MPU6050_ADDRESS       		0x68

// Registradores
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG    	0x1C
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I    		0x75

// BITS
#define MPU6050_WHO_AM_I_BIT       			6

#define MPU6050_PWR1_CLKSEL_BIT     		2
#define MPU6050_PWR1_CLKSEL_LENGTH  		3

#define MPU6050_GCONFIG_FS_SEL_BIT      	4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   	2

#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1


// PWR_MGMT_1 - clock source setting
#define MPU6050_CLOCK_INTERNAL    	0x00
#define MPU6050_CLOCK_PLL_XGYRO     0x01
#define MPU6050_CLOCK_PLL_YGYRO    	0x02
#define MPU6050_CLOCK_PLL_ZGYRO    	0x03
#define MPU6050_CLOCK_PLL_EXT32K   	0x04
#define MPU6050_CLOCK_PLL_EXT19M   	0x05
#define MPU6050_CLOCK_KEEP_RESET   	0x07

// MPU6050_RA_CONFIG
#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

// GYRO_CONFIG - scale gyroscope range
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

// ACCEL_CONFIG - scale accelerometer range
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

// USER_CTRL
#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0





uint8_t mpu6050_detect(void); 		// Verificar a presença do MPU6050
//void mpu6050_calibrate(float*, float*); // Calibração do MPU6050
void mpu6050_config(void);			// Configuração do MPU6050
void mpu6050_getAcceleration(int16_t*, int16_t*, int16_t*);
void mpu6050_getRotation(int16_t*, int16_t*, int16_t*);
void mpu6050_getTemperature(int16_t*);

void mpu6050_initialize(void); // Inicialização
void mpu6050_reset(void);
void mpu6050_setSleepEnabled(uint8_t);
void mpu6050_setMemoryBank(uint8_t, uint8_t, uint8_t);
int16_t mpu6050_getXGyroOffset(void);
int16_t mpu6050_getYGyroOffset(void);
int16_t mpu6050_getZGyroOffset(void);
void mpu6050_setI2CBypassEnabled(uint8_t);
void mpu6050_setI2CMasterModeEnabled(uint8_t);
void mpu6050_getMotion6(int16_t*, int16_t*, int16_t*, int16_t*, int16_t*, int16_t*);

#endif /* I2C_MPU6050_MPU6050_H_ */
