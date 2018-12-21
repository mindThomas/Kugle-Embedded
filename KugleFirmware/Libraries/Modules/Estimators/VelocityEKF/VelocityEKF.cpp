/* Copyright (C) 2018 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  thomasj@tkjelectronics.com
 * ------------------------------------------
 */
 
#include "VelocityEKF.h"
#include "VelocityEstimator.h"
#include "VelocityEstimator_initialize.h"

#include "Parameters.h"
#include <string.h> // for memcpy

VelocityEKF::VelocityEKF(Parameters& params) : _params(params)
{
	Reset();
}

VelocityEKF::~VelocityEKF()
{
}

void VelocityEKF::Reset()
{
	VelocityEstimator_initialize(_params.estimator.VelocityEstimator_P_init_diagonal, X, P);
}

void VelocityEKF::Step(const float dt, const float EncoderDiffMeas[3], const float qEst[4], const float Cov_qEst[4*4], const float qDotEst[4], const float COMest[3])
{
	float X_old[sizeof(X)];
	float P_old[sizeof(P)];

	// Copy previous state into old temporary variables to be used as input
	memcpy(X_old, X, sizeof(X_old));
	memcpy(P_old, P, sizeof(X_old));

    VelocityEstimator(X_old, P_old,
      EncoderDiffMeas,
      qEst, Cov_qEst, qDotEst,
      dt,
      _params.model.i_gear, _params.model.TicksPrRev,
	  _params.model.Jk, _params.model.Mk, _params.model.rk, _params.model.Mb, _params.model.Jbx, _params.model.Jby, _params.model.Jbz, _params.model.Jw, _params.model.rw, _params.model.Bvk, _params.model.Bvm, _params.model.Bvb, _params.model.l, _params.model.g,
	  COMest,
      1E-5, // Var_COM
      10.0f, // eta_qQEKF_velocity
      0.0f, // eta_dqQEKF_encoder
      X, P);
}
