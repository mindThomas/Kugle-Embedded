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
 
#include "QEKF.h"
#include "QEKF_coder.h"
#include "QEKF_initialize.h"
#include "Math.h"
#include <string.h> // for memcpy
 
#include "Quaternion.h"

QEKF::QEKF(Parameters& params, Timer * microsTimer) : _params(params), _microsTimer(microsTimer)
{
	Reset();
}

QEKF::QEKF(Parameters& params) : _params(params), _microsTimer(0)
{
	Reset();
}

QEKF::~QEKF()
{
}

void QEKF::Reset()
{
	QEKF_initialize(_params.estimator.QEKF_P_init_diagonal, X, P);

	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;
}

/**
 * @brief 	Reset attitude estimator to an angle based on an accelerometer measurement
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 */
void QEKF::Reset(const float accelerometer[3])
{
	Reset();
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3])
{
	Step(accelerometer, gyroscope, _params.estimator.EstimateBias);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias)
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaMicros(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	Step(accelerometer, gyroscope, EstimateBias, dt);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements and passed time
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
 * @param	dt    			   Input: time passed since last estimate
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias, const float dt)
{
	Step(accelerometer, gyroscope, EstimateBias, _params.estimator.cov_acc_mpu, _params.estimator.cov_gyro_mpu, _params.estimator.sigma2_bias, _params.model.g, dt);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements and passed time
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated*
 * @param   cov_acc            Input: accelerometer sensor covariance matrix
 * @param   cov_gyro           Input: gyroscope sensor covariance matrix
 * @param   sigma2_bias        Input: bias variance (random walk)
 * @param   g                  Input: gravity constant [m/s^2]
 * @param	dt    			   Input: time passed since last estimate
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias, const float cov_acc[9], const float cov_gyro[9], const float sigma2_bias, const float g, const float dt)
{
	if (dt == 0) return; // no time has passed

	float X_prev[10];
	memcpy(X_prev, X, sizeof(X_prev));

	float P_prev[10*10];
	memcpy(P_prev, P, sizeof(P_prev));

	_QEKF(X_prev, P_prev,
		 gyroscope, accelerometer,
		 dt,
		 EstimateBias,
		 true,  // normalize accelerometer = true
		 cov_gyro, cov_acc, sigma2_bias,
		 g,
		 X, P);

    if (_params.estimator.CreateQdotFromQDifference) {
      X[4] = (X[0] - X_prev[0]) / dt; // dq[0]
      X[5] = (X[1] - X_prev[1]) / dt; // dq[1]
      X[6] = (X[2] - X_prev[2]) / dt; // dq[2]
      X[7] = (X[3] - X_prev[3]) / dt; // dq[3]
    }
}

/**
 * @brief 	Get estimated attitude quaternion
 * @param	q[4]		Output: estimated attitude quaternion
 */
void QEKF::GetQuaternion(float q[4])
{
	q[0] = X[0];
	q[1] = X[1];
	q[2] = X[2];
	q[3] = X[3];
}

/**
 * @brief 	Get estimated attitude quaternion derivative
 * @param	dq[4]		Output: estimated attitude quaternion derivative
 */
void QEKF::GetQuaternionDerivative(float dq[4])
{
	dq[0] = X[4];
	dq[1] = X[5];
	dq[2] = X[6];
	dq[3] = X[7];
}

/**
 * @brief 	Get covariance matrix of estimated quaternion
 * @param	Cov_q[4*4]		Output: quaternion estimate covariance
 */
void QEKF::GetQuaternionCovariance(float Cov_q[4*4])
{
    for (int m = 0; m < 4; m++) {
      for (int n = 0; n < 4; n++) {
        Cov_q[4*m + n] = P[10*m + n];
      }
    }
}

/**
 * @brief 	Get covariance matrix of estimated quaternion derivative
 * @param	Cov_dq[4*4]		Output: quaternion derivative estimate covariance
 */
void QEKF::GetQuaternionDerivativeCovariance(float Cov_dq[4*4])
{
    for (int m = 0; m < 4; m++) {
      for (int n = 0; n < 4; n++) {
        Cov_dq[4*m + n] = P[10*m + n + 44];
      }
    }
}

bool QEKF::UnitTest(void)
{
	const float g = 9.82f;

	const float cov_gyro_mpu[9] = {0.2529E-03,   -0.0064E-03,    0.1981E-03,
								  -0.0064E-03,    0.9379E-03,   -0.0038E-03,
								   0.1981E-03,   -0.0038E-03,    1.6828E-03};
	const float cov_acc_mpu[9] = {0.4273E-03,    0.0072E-03,    0.0096E-03,
								  0.0072E-03,    0.4333E-03,    0.0041E-03,
								  0.0096E-03,    0.0041E-03,    1.0326E-03};

	const float sigma2_bias = 1E-11;
	const bool EstimateBias = true;

	const float QEKF_P_init_diagonal[10] = {1E-5, 1E-5, 1E-5, 1E-7,   1E-7, 1E-7, 1E-7, 1E-7,   1E-5, 1E-5};

	QEKF_initialize(QEKF_P_init_diagonal, X, P); // reset

	float Accelerometer[3] = {0.05, 0, 9.82};
	float Gyroscope[3] = {1.0, 0.5, 0.09};

	float dt = 1.0 / 400.0; // 400 Hz

	Step(Accelerometer, Gyroscope, EstimateBias, cov_acc_mpu, cov_gyro_mpu, sigma2_bias, g, dt);
	Step(Accelerometer, Gyroscope, EstimateBias, cov_acc_mpu, cov_gyro_mpu, sigma2_bias, g, dt);

	float q[4];
	GetQuaternion(q);

	float dq[4];
	GetQuaternionDerivative(dq);

	float Cov_q[4*4];
	GetQuaternionCovariance(Cov_q);

	float bias[2];
	bias[0] = X[8];
	bias[1] = X[9];

	float q_expected[4] = {999.9993e-03, 1.1463e-03, 0.1727e-03, 0.1130e-03};
	if (!(Math_Round(q[0], 3) == Math_Round(q_expected[0], 3) &&
		  Math_Round(q[1], 7) == Math_Round(q_expected[1], 7) &&
		  Math_Round(q[2], 7) == Math_Round(q_expected[2], 7) &&
		  Math_Round(q[3], 7) == Math_Round(q_expected[3], 7)))
		return false;

	float dq_expected[4] = {0.1649e-03, 499.9784e-03, 250.0039e-03, 45.2001e-03};
	if (!(Math_Round(dq[0], 7) == Math_Round(dq_expected[0], 7) &&
		  Math_Round(dq[1], 7) == Math_Round(dq_expected[1], 7) &&
		  Math_Round(dq[2], 7) == Math_Round(dq_expected[2], 7) &&
		  Math_Round(dq[3], 7) == Math_Round(dq_expected[3], 7)))
		return false;

	float bias_expected[2] = {1.3754e-07, 3.2047e-07};
	if (!(Math_Round(bias[0], 10) == Math_Round(bias_expected[0], 10) &&
		  Math_Round(bias[1], 10) == Math_Round(bias_expected[1], 10)))
		return false;

	float Cov_q_diag_expected[4] = {0.9281e-05, 0.8441e-05, 0.8424e-05, 0.0103e-05};
	if (!(Math_Round(Cov_q[0], 9) == Math_Round(Cov_q_diag_expected[0], 9) &&
		  Math_Round(Cov_q[1+4], 9) == Math_Round(Cov_q_diag_expected[1], 9) &&
		  Math_Round(Cov_q[2+8], 9) == Math_Round(Cov_q_diag_expected[2], 9) &&
		  Math_Round(Cov_q[3+12], 9) == Math_Round(Cov_q_diag_expected[3], 9)))
		return false;

	return false;

	return true;
}
