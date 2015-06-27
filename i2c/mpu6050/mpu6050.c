/*
 * mpu6050.c
 *
 *  Created on: 09/03/2015
 *      Author: Victor
 */

#include "mpu6050.h"
#include "../twi_master.h"
#include "../../setup.h"

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
uint8_t mpu6050_detect(void) {
	uint8_t data = 0;
	twi_master_readRegister(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, &data, sizeof(data));
	return read_bits(data, 6, 6) == 0x34;
}


/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void mpu6050_setClockSource(uint8_t data) {
	twi_master_writeBits(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, data);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void mpu6050_setFullScaleGyroRange(uint8_t data) {
	twi_master_writeBits(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, data);
}

/*
 * Clear SelfTest to gyroscope [7:4]
 */
void mpu6050_clearGyroSelfTest() {
	uint8_t b = 0;
	twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, &b, 1);
}


/** Set full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
void mpu6050_setFullScaleAccelRange(uint8_t data) {
	twi_master_writeBits(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, data);
}

/*
 * Clear SelfTest to accelerometer [7:4]
 */
void mpu6050_clearAccelSelfTest() {
	uint8_t b = 0;
	twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &b, 1);
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void mpu6050_setSleepEnabled(uint8_t enabled) {
	twi_master_writeBit(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/** Set wake cycle enabled status.
 * @param enabled New sleep mode enabled status
 * @see getWakeCycleEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CYCLE_BIT
 */
void mpu6050_setWakeCycleEnabled(uint8_t enabled) {
	twi_master_writeBit(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled);
}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void mpu6050_reset() {
	twi_master_writeBit(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

/** Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @param enabled New I2C bypass enabled status
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_I2C_BYPASS_EN_BIT
 */
void mpu6050_setI2CBypassEnabled(uint8_t enabled) {
	twi_master_writeBit(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/** Set external FSYNC configuration.
  * Configures the external Frame Synchronization (FSYNC) pin sampling. An
 * external signal connected to the FSYNC pin can be sampled by configuring
 * EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
 * strobes may be captured. The latched FSYNC signal will be sampled at the
 * Sampling Rate, as defined in register 25. After sampling, the latch will
 * reset to the current FSYNC signal state.
 *
 * The sampled value will be reported in place of the least significant bit in
 * a sensor data register determined by the value of EXT_SYNC_SET according to
 * the following table.
 *
 * <pre>
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 * 0            | Input disabled
 * 1            | TEMP_OUT_L[0]
 * 2            | GYRO_XOUT_L[0]
 * 3            | GYRO_YOUT_L[0]
 * 4            | GYRO_ZOUT_L[0]
 * 5            | ACCEL_XOUT_L[0]
 * 6            | ACCEL_YOUT_L[0]
 * 7            | ACCEL_ZOUT_L[0]
 * </pre>
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void mpu6050_setExternalFrameSync(uint8_t sync) {
	twi_master_writeBits(MPU6050_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

/** Set digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void mpu6050_setDLPFMode(uint8_t mode) {
	twi_master_writeBits(MPU6050_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/** Set gyroscope sample rate divider.
 *
 * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
 * Motion detection, and Free Fall detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6050 Product Specification document.
 *
 * @param rate New sample rate divider
 * @return Current sample rate
 * @see MPU6050_RA_SMPLRT_DIV
 */
void mpu6050_setRate(uint8_t rate) {
	twi_master_writeBits(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 7, 8, rate);
}


/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void mpu6050_getAcceleration(int16_t* ax, int16_t* ay, int16_t* az) {
	uint8_t buffer[6];
	twi_master_readRegister(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, buffer, sizeof(buffer));
	*ax = (((int16_t)buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t)buffer[2]) << 8) | buffer[3];
	*az = (((int16_t)buffer[4]) << 8) | buffer[5];
}

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void mpu6050_getRotation(int16_t* gx, int16_t* gy, int16_t* gz) {
	uint8_t buffer[6];
	twi_master_readRegister(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, buffer, sizeof(buffer));
    *gx = (((int16_t)buffer[0]) << 8) | buffer[1];
    *gy = (((int16_t)buffer[2]) << 8) | buffer[3];
    *gz = (((int16_t)buffer[4]) << 8) | buffer[5];
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void mpu6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
	uint8_t buffer[14];
	twi_master_readRegister(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, buffer, sizeof(buffer));
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

/** Get current internal temperature.
 * @return Temperature reading in 16-bit 2's complement format
 * @see MPU6050_RA_TEMP_OUT_H
 */
void mpu6050_getTemperature(int16_t* temp) {
	uint8_t buffer[2];
	twi_master_readRegister(MPU6050_ADDRESS, MPU6050_RA_TEMP_OUT_H, buffer, sizeof(buffer));
    *temp = (((int16_t)buffer[0]) << 8) | buffer[1];
}

void mpu6050_disableAllFIFO() {
	uint8_t b = 0;
	twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, &b, 1);
}

/** Set FIFO enabled status.
 * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
 * cannot be written to or read from while disabled. The FIFO buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 * @return Current FIFO enabled status
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void mpu6050_setFIFOEnabled(uint8_t enabled) {
	twi_master_writeBit(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 *
 * When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
 * external sensor slave devices on the auxiliary I2C bus. When this bit is
 * cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
 * driven by the primary I2C bus (SDA and SCL). This is a precondition to
 * enabling Bypass Mode. For further information regarding Bypass Mode, please
 * refer to Register 55.
 *
 * @return Current I2C Master Mode enabled status
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
void mpu6050_setI2CMasterModeEnabled(uint8_t enabled) {
	twi_master_writeBit(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void mpu6050_resetFIFO() {
	twi_master_writeBit(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

void mpu6050_resetDMP() {
	twi_master_writeBit(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}

void mpu6050_enableonlyGyroAccelFIFO() {
	uint8_t b = 0x78;
	twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, &b, 1);
}

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t mpu6050_getFIFOCount() {
    return twi_master_read16(MPU6050_ADDRESS, MPU6050_RA_FIFO_COUNTH);
}

void mpu6050_getFIFOBytes(uint8_t *data, uint8_t length) {
	twi_master_readRegister(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, data, length);
}

int16_t mpu6050_getXAccelOffset() {
	return twi_master_read16(MPU6050_ADDRESS, MPU6050_RA_XA_OFFS_H);
}

int16_t mpu6050_getYAccelOffset() {
	return twi_master_read16(MPU6050_ADDRESS, MPU6050_RA_YA_OFFS_H);
}

int16_t mpu6050_getZAccelOffset() {
	return twi_master_read16(MPU6050_ADDRESS, MPU6050_RA_ZA_OFFS_H);
}

int16_t mpu6050_getXGyroOffset() {
	return twi_master_read16(MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH);
}

int16_t mpu6050_getYGyroOffset() {
	return twi_master_read16(MPU6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH);
}

int16_t mpu6050_getZGyroOffset() {
	return twi_master_read16(MPU6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH);
}



/**
 * Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
 * of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
 *
 * - http://courses.cs.washington.edu/courses/cse466/14au/labs/l4/MPU6050BasicExample.ino
 * - https://github.com/kriswiner/MPU6050HMC5883AHRS/blob/master/MPU6050HMC5883AHRS.ino#L221
 * - https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp
 * - https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.h
 */
/*void mpu6050_calibrate(float * gBias, float * aBias) {
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	mpu6050_reset();

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	mpu6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	delay(200);

	// Configure device for bias calculation
	mpu6050_disableAllFIFO();									// Disable FIFO
	mpu6050_setClockSource(MPU6050_CLOCK_INTERNAL);		// Turn on internal clock source
	mpu6050_setFIFOEnabled(0);							// Disable FIFO mode
	mpu6050_setI2CMasterModeEnabled(0);					// I2C master mode
	mpu6050_resetFIFO();								// Reset FIFO
	mpu6050_resetDMP();									// Reset DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	mpu6050_setDLPFMode(MPU6050_DLPF_BW_188);		// Set low-pass filter to 188 Hz
	mpu6050_setRate(0x00);							// Set sample rate to 1 kHz
	mpu6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	mpu6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // Set accelerometer full-scale to 2 g, maximum sensitivity

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	mpu6050_setFIFOEnabled(1);	// Enable FIFO mode
	mpu6050_enableonlyGyroAccelFIFO();  // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
	delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	mpu6050_disableAllFIFO();	// Disable gyro and accelerometer sensors for FIFO
	fifo_count = mpu6050_getFIFOCount();
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	// Read packets
	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		mpu6050_getFIFOBytes(data, 12); // read data for averaging

		// Form signed 16-bit integer for each sample in FIFO
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[0] += (int32_t) accel_temp[0];
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	// Normalize sums to get average count biases
	accel_bias[0] /= (int32_t) packet_count;
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	// Remove gravity from the z-axis accelerometer bias calculation
	float resolution = (float) ACCEL_AFS / (float) RESOLUTION_16BIT;
	if(accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) resolution;
	}
	else {
		accel_bias[2] += (int32_t) resolution;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH, &data[0], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRL, &data[1], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH, &data[2], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_YG_OFFS_USRL, &data[3], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, &data[4], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRL, &data[5], 1);

	resolution = (float) GYRO_DPS / (float) RESOLUTION_16BIT;
	gBias[0] = (float) gyro_bias[0] * resolution; // construct gyro bias in deg/s for later manual subtraction
	gBias[1] = (float) gyro_bias[1] * resolution;
	gBias[2] = (float) gyro_bias[2] * resolution;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.
	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	accel_bias_reg[0] = mpu6050_getXAccelOffset();
	accel_bias_reg[1] = mpu6050_getYAccelOffset();
	accel_bias_reg[2] = mpu6050_getZAccelOffset();

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
	  if(accel_bias_reg[ii] & mask)
		  mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers


	// Push accelerometer biases to hardware registers
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_XA_OFFS_H, &data[0], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_XA_OFFS_L_TC, &data[1], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_YA_OFFS_H, &data[2], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_YA_OFFS_L_TC, &data[3], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_YA_OFFS_H, &data[4], 1);
	//twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_ZA_OFFS_L_TC, &data[5], 1);

	resolution = (float) ACCEL_AFS / (float) RESOLUTION_16BIT;
	aBias[0] = (float)accel_bias[0] * resolution;
	aBias[1] = (float)accel_bias[1] * resolution;
	aBias[2] = (float)accel_bias[2] * resolution;

}*/

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void mpu6050_config(void) {

	// wake up device
	mpu6050_setClockSource(MPU6050_CLOCK_INTERNAL);
	mpu6050_setSleepEnabled(0);
	mpu6050_setWakeCycleEnabled(0);
	delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	mpu6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    // Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	mpu6050_setExternalFrameSync(MPU6050_EXT_SYNC_DISABLED);
	mpu6050_setDLPFMode(MPU6050_DLPF_BW_42);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	mpu6050_setRate(0x04); // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	mpu6050_clearGyroSelfTest();
	mpu6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250);

	// Set accelerometer configuration
	mpu6050_clearAccelSelfTest();
	mpu6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	mpu6050_setI2CBypassEnabled(1);


}

void mpu6050_initialize() {
	mpu6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	mpu6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	mpu6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	mpu6050_setSleepEnabled(0);
}

void mpu6050_setMemoryBank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    twi_master_writeRegister(MPU6050_ADDRESS, MPU6050_RA_BANK_SEL, &bank, 1);
}
