/*
 * madgwick.h
 *
 *  Created on: 04/05/2015
 *      Author: Victor
 *
 *  Referencias:
 *  - https://github.com/kriswiner/MPU6050HMC5883AHRS/blob/master/quaternion%20Filters.ino
 *  - https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU/blob/master/x-IMU%20IMU%20and%20AHRS%20Algorithms/x-IMU%20IMU%20and%20AHRS%20Algorithms/AHRS/MadgwickAHRS.cs
 */

#ifndef LIBRARIES_QUATERNION_MADGWICK_H_
#define LIBRARIES_QUATERNION_MADGWICK_H_

typedef struct {
	float samplePeriod;		// Sample period
	float beta;				// Gain beta
	float quaternion[4];	// Quaternion
} madgwick_params;

void madgwick_init(madgwick_params* p, float samplePeriod, float beta);
void madgwick_update(madgwick_params* p, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#endif /* LIBRARIES_QUATERNION_MADGWICK_H_ */
