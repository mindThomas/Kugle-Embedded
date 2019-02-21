/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details. 
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#ifndef DEVICES_IMU_H
#define DEVICES_IMU_H

#include "cmsis_os.h"
#include "EEPROM.h"

class IMU
{
	public:
		typedef struct Measurement_t {
			float Accelerometer[3];
			float Gyroscope[3];
			float Magnetometer[3];
		} Measurement_t;

		typedef struct Estimates_t {
			float q[4];
			float dq[4];
		} Estimates_t;

		typedef struct calibration_t {
			bool calibrated = false;
			float imu_calibration_matrix[9];
			float gyro_bias[3];
		};

	public:
		virtual ~IMU() {};

		virtual uint32_t WaitForNewData(uint32_t xTicksToWait = portMAX_DELAY) { return pdFALSE; };
		virtual void Get(Measurement_t& measurement) {};
		virtual void GetEstimates(Estimates_t& estimates) {}; // if supported, eg. by Xsens IMU

		void Calibrate(bool storeInEEPROM = true);
		void CorrectMeasurement(Measurement_t& measurement, bool correctGyroBias = true, bool correctAlignment = true);

		void AttachEEPROM(EEPROM * eeprom);

	private:
		void LoadCalibrationFromEEPROM(void);
		void ValidateCalibration(void);
		float vector_length(const float v[3]);
		void calibrateImu(const float desired_acc_vector[3], const float actual_acc_vector[3], float calibration_matrix[9]);
		void adjustImuMeasurement(float& gx, float& gy, float& gz, float& ax, float& ay, float& az, const float calibration_matrix[9], const float gyro_bias[3]);

	private:
		EEPROM * eeprom_ = 0;
		calibration_t calibration_;

		const float reference_acc_vector_[3] = {0.0f, 0.0f, 9.82f};

};
	
	
#endif
