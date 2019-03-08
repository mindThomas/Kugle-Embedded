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
 
#include "MPC.h"

#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "acado_indices.h"

#include "Debug.h"
#include "Math.h"
#include "Quaternion.h"


/** Instance of ACADO data structure */
extern "C" __EXPORT {
	// Put the ACADO variables and workspace in same memory region as the FreeRTOS heap is put but WITHOUT OVERLAPPING ---- This is handled by the compiler/linker
	__attribute__ ((used,section(".ramd1_acadoVariables"))) ACADOvariables acadoVariables;
	__attribute__ ((used,section(".ramd1_acadoWorkspace"))) ACADOworkspace acadoWorkspace;
}

namespace MPC {

    ACADO_t& MPC::ACADO = *((ACADO_t *)&acadoVariables);

    const double MPC::WPathFollow = 100.0;
    const double MPC::WVelocity = 2.0;
    const double MPC::WSmoothness = 10.0;

    // Horizon weight matrix (cost)
    const double MPC::Wdiag[ACADO_NY] = {MPC::WPathFollow, MPC::WPathFollow,  0.1,0.1,  0.1,0.1,  999.0,MPC::WVelocity,  MPC::WSmoothness,MPC::WSmoothness,  99999.0,99999999.0}; // { x_err;y_err; q2;q3;  omega_ref_x;omega_ref_y;  velocity_matching; velocity_error;   domega_ref_x;domega_ref_y;   velocity_slack_variable;angle_slack_variable;proximity_slack_variable }

    // Final state weight matrix (cost)
    const double MPC::WNdiag[ACADO_NYN] = {MPC::WPathFollow, MPC::WPathFollow,  1000*0.1,1000*0.1,  1000*0.1,1000*0.1,  0*999.0,0*MPC::WVelocity}; // { x_err;y_err; q2;q3;  omega_ref_x;omega_ref_y;  velocity_matching; velocity_error }


    MPC::MPC()
    {
        if (sizeof(ACADO_t) != sizeof(ACADOvariables)) {
            ERROR("[ERROR] MPC ACADO variable typedefinition does not match generated!\n");
        }

        // Initialize ACADO
        resetACADO();
        resetStates();

        // Set window and path parameters
        WindowWidth_ = 5.0;
        WindowHeight_ = 5.0;
        WindowOffset_[0] = 0;
        WindowOffset_[1] = 0;
        WindowOrientation_ = 0;
        pathApproximationOrder_ = 8;
        WindowOrientationSelection_ = INERTIAL_FRAME;

        // Set weights
        setWeightsDiag(Wdiag, sizeof(Wdiag)/sizeof(Wdiag[0]), WNdiag, sizeof(WNdiag)/sizeof(WNdiag[0]));

        // Set internal to default values
        setVelocityBounds(0.0, 3.0);
        setDesiredVelocity(1.0);
        setControlLimits(deg2rad(30), deg2rad(10));
        setReferences();

        // Set an empty reference path
        Path emptyPath;
        setPath(emptyPath, (const double[2]){0,0}, (const double[2]){0,0});

        // Set no obstacles
        std::vector<Obstacle> noObstacles;
        setObstacles(noObstacles);
    }


    MPC::~MPC()
    {

    }

    void MPC::Reset(void)
    {
        resetACADO();
        resetStates();

        // Set internal parameters
        setVelocityBounds(minVelocity_, maxVelocity_);
        setDesiredVelocity(desiredVelocity_);
        setControlLimits(maxAngularVelocity_, maxAngle_);
        setReferences();

        // Set current path and states
        //setPath(windowPath_, windowPathOrigin_, windowPathOrigin_);
        //setCurrentState(position_, velocity_, quaternion_);

        // Set an empty reference path
        Path emptyPath;
        setPath(emptyPath, (const double[2]){0,0}, (const double[2]){0,0});

        // Set no obstacles
        std::vector<Obstacle> noObstacles;
        setObstacles(noObstacles);
    }

    Path MPC::getCurrentPath(void)
    {
        return windowPath_;
    }

    double MPC::getClosestPointOnPath(void)
    {
        return closestPositionOnCurrentPathToOrigin_;
    }

    void MPC::setPath(Path& path, const double origin[2], const double currentPosition[2])
    {
        windowPath_ = path;
        windowPathOrigin_[0] = origin[0];
        windowPathOrigin_[1] = origin[1];
        windowPathLength_ = path.length();

        if (path.order() > 9) {
            ERROR("ERROR: Path order not supported by MPC (too high)");
            return;
        }

        // Based on current position, compute the closest position (s-value) on this new path
        Point currentPositionInWindow;
        currentPositionInWindow.x = currentPosition[0] - windowPathOrigin_[0];
        currentPositionInWindow.y = currentPosition[1] - windowPathOrigin_[1];
        closestPositionOnCurrentPathToOrigin_ = windowPath_.FindClosestPoint(currentPositionInWindow);

        Debug::print("MPC path updated:\n");
        Debug::printf("   length = %.2f\n", windowPathLength_);
        Debug::printf("   closest point = %.2f\n", closestPositionOnCurrentPathToOrigin_);
        windowPath_.print();

        // Set path for full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].cx0 = windowPath_.getXcoefficient(0);
            ACADO.od[i].cx1 = windowPath_.getXcoefficient(1);
            ACADO.od[i].cx2 = windowPath_.getXcoefficient(2);
            ACADO.od[i].cx3 = windowPath_.getXcoefficient(3);
            ACADO.od[i].cx4 = windowPath_.getXcoefficient(4);
            ACADO.od[i].cx5 = windowPath_.getXcoefficient(5);
            ACADO.od[i].cx6 = windowPath_.getXcoefficient(6);
            ACADO.od[i].cx7 = windowPath_.getXcoefficient(7);
            ACADO.od[i].cx8 = windowPath_.getXcoefficient(8);
            ACADO.od[i].cx9 = windowPath_.getXcoefficient(9);

            ACADO.od[i].cy0 = windowPath_.getYcoefficient(0);
            ACADO.od[i].cy1 = windowPath_.getYcoefficient(1);
            ACADO.od[i].cy2 = windowPath_.getYcoefficient(2);
            ACADO.od[i].cy3 = windowPath_.getYcoefficient(3);
            ACADO.od[i].cy4 = windowPath_.getYcoefficient(4);
            ACADO.od[i].cy5 = windowPath_.getYcoefficient(5);
            ACADO.od[i].cy6 = windowPath_.getYcoefficient(6);
            ACADO.od[i].cy7 = windowPath_.getYcoefficient(7);
            ACADO.od[i].cy8 = windowPath_.getYcoefficient(8);
            ACADO.od[i].cy9 = windowPath_.getYcoefficient(9);

            ACADO.od[i].trajectoryLength = windowPathLength_;
            ACADO.od[i].trajectoryStart = closestPositionOnCurrentPathToOrigin_;
        }

        // Reset horizon predictions for the movement on the path (s-value)
        //real_t s_reset = ACADO.x[0].s;
        real_t s_reset = ACADO.x[1].s; // resetting such that the value 1 step in the horizon becomes 0, since the states has not been shifted yet
        real_t x_reset = ACADO.x[1].x;
        real_t y_reset = ACADO.x[1].y;
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.x[i].s = ACADO.x[i].s - s_reset;
            ACADO.x[i].x = ACADO.x[i].x - x_reset;
            ACADO.x[i].y = ACADO.x[i].y - y_reset;
        }
        // This results in ACADO.x[0].s = 0 and the path value predictions are then increasing from there

        if ((ACADO.x[ACADO_N].s + closestPositionOnCurrentPathToOrigin_) > windowPathLength_) {
            // Current MPC horizon initialization is running outside of new path. Reset to evenly space the s-values along path
            double s_value = 0;
            double s_spacing = (windowPathLength_ - closestPositionOnCurrentPathToOrigin_) / (ACADO_N-1);
            for (unsigned int i = 1; i < (ACADO_N+1); i++) {
                ACADO.x[i].s = s_value;
                s_value += s_spacing;
            }
        }
    }

    void MPC::setXYreferencePosition(const double position[2])
    {
        // OBS. Sending discrete position references to the path following controller turns it into a "trajectory tracking" controller
        Path emptyPath;
        windowPath_ = emptyPath;
        windowPathOrigin_[0] = position[0];
        windowPathOrigin_[1] = position[1];
        windowPathLength_ = 99; // arbitrary length, since it is just a constant point

        Debug::printf("MPC static position enabled: \n   x = %.2f\n   y = %.2f\n", position[0], position[1]);

        // Set path for full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].cx0 = 0;
            ACADO.od[i].cx1 = 1000; // OBS. Even though we just want to hold the position, we still need to set a valid path due to the way the MPC tries to take the derivative
            ACADO.od[i].cx2 = 0;
            ACADO.od[i].cx3 = 0;
            ACADO.od[i].cx4 = 0;
            ACADO.od[i].cx5 = 0;
            ACADO.od[i].cx6 = 0;
            ACADO.od[i].cx7 = 0;
            ACADO.od[i].cx8 = 0;
            ACADO.od[i].cx9 = 0;

            ACADO.od[i].cy0 = 0;
            ACADO.od[i].cy1 = 0;
            ACADO.od[i].cy2 = 0;
            ACADO.od[i].cy3 = 0;
            ACADO.od[i].cy4 = 0;
            ACADO.od[i].cy5 = 0;
            ACADO.od[i].cy6 = 0;
            ACADO.od[i].cy7 = 0;
            ACADO.od[i].cy8 = 0;
            ACADO.od[i].cy9 = 0;

            ACADO.od[i].trajectoryLength = windowPathLength_;
            ACADO.od[i].trajectoryStart = 0;
        }

        // Reset horizon predictions for the movement on the path (s-value)
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.x[i].s = 0;
            ACADO.x[i].x = 0;
            ACADO.x[i].y = 0;
        }
    }

    void MPC::setObstacles(std::vector<Obstacle>& obstacles)
    {
        currentObstacles_ = obstacles;

        // Sort the obstacles to extract and use the nearest obstacles
        // SORTING NOT IMPLEMENTED ON EMBEDDED PROCESSOR
        //std::sort(currentObstacles_.begin(), currentObstacles_.end(), std::bind(&Obstacle::proximitySorting, windowPathOrigin_, std::placeholders::_1, std::placeholders::_2));

        // Set obstacles throughout full horizon
        double obstacle[2];
        double obstacle_in_window[2];

        if (currentObstacles_.size() >= 1) {
            obstacle[0] = currentObstacles_.at(0).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(0).y - windowPathOrigin_[1];
            Math_Rotate2D(-WindowOrientation_, obstacle, obstacle_in_window);
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs1_x = obstacle_in_window[0];
                ACADO.od[i].obs1_y = obstacle_in_window[1];
                ACADO.od[i].obs1_r = currentObstacles_.at(0).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs1_x = 99;
                ACADO.od[i].obs1_y = 99;
                ACADO.od[i].obs1_r = 0.001;
            }
        }
        /*
        if (currentObstacles_.size() >= 2) {
            obstacle[0] = currentObstacles_.at(1).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(1).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            obstacle_in_window = rot * obstacle;
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs2_x = obstacle_in_window[0];
                ACADO.od[i].obs2_y = obstacle_in_window[1];
                ACADO.od[i].obs2_r = currentObstacles_.at(1).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs2_x = 99;
                ACADO.od[i].obs2_y = 99;
                ACADO.od[i].obs2_r = 0.001;
            }
        }
        if (obstacles.size() >= 3) {
            obstacle[0] = currentObstacles_.at(2).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(2).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            obstacle_in_window = rot * obstacle;
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs3_x = obstacle_in_window[0];
                ACADO.od[i].obs3_y = obstacle_in_window[1];
                ACADO.od[i].obs3_r = currentObstacles_.at(2).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs3_x = 99;
                ACADO.od[i].obs3_y = 99;
                ACADO.od[i].obs3_r = 0.001;
            }
        }
        if (currentObstacles_.size() >= 4) {
            obstacle[0] = currentObstacles_.at(3).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(3).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            obstacle_in_window = rot * obstacle;
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs4_x = obstacle_in_window[0];
                ACADO.od[i].obs4_y = obstacle_in_window[1];
                ACADO.od[i].obs4_r = currentObstacles_.at(3).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs4_x = 99;
                ACADO.od[i].obs4_y = 99;
                ACADO.od[i].obs4_r = 0.001;
            }
        }
        if (currentObstacles_.size() >= 5) {
            obstacle[0] = currentObstacles_.at(4).x - windowPathOrigin_[0];
            obstacle[1] = currentObstacles_.at(4).y - windowPathOrigin_[1];
            Eigen::Matrix2d rot = Eigen::Rotation2Dd(-WindowOrientation_).toRotationMatrix();
            obstacle_in_window = rot * obstacle;
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs5_x = obstacle_in_window[0];
                ACADO.od[i].obs5_y = obstacle_in_window[1];
                ACADO.od[i].obs5_r = currentObstacles_.at(4).radius + RobotRadius; // inflate radius with robot radius, to ensure sufficient space for avoidance
            }
        } else {
            for (unsigned int i = 0; i < (ACADO_N+1); i++) {
                ACADO.od[i].obs5_x = 99;
                ACADO.od[i].obs5_y = 99;
                ACADO.od[i].obs5_r = 0.001;
            }
        }*/
    }

    void MPC::setVelocityBounds(double min_velocity, double max_velocity)
    {
        minVelocity_ = min_velocity;
        maxVelocity_ = max_velocity;

        // Set bounds for the full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            //ACADO.od[i].minVelocity = minVelocity_; // minimum velocity ignored for now
            ACADO.od[i].maxVelocity = maxVelocity_;
        }
    }

    void MPC::setDesiredVelocity(double velocity)
    {
        desiredVelocity_ = velocity;

        // Set the desired velocity for the full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].desiredVelocity = desiredVelocity_;
        }
    }

    void MPC::setControlLimits(double maxAngularVelocity, double maxAngle)
    {
        maxAngularVelocity_ = maxAngularVelocity;
        maxAngle_ = maxAngle;

        // Set bounds for the full horizon
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.od[i].maxAngle = maxAngle_;
            ACADO.od[i].maxOmegaRef = maxAngularVelocity_;
        }
    }

    void MPC::setWeightsDiag(const double * Wdiag, unsigned int numWdiag, const double * WNdiag, unsigned int numWNdiag)
    {
        if (numWdiag != ACADO_NY || numWdiag != ACADO_NY) {
        	ERROR("[ERROR] Incorrect W weight matrix for MPC");
            return; // Error, weights of incorrect dimension
        }
        if (numWNdiag != ACADO_NYN || numWNdiag != ACADO_NYN) {
        	ERROR("[ERROR] Incorrect WN weight matrix for MPC");
            return; // Error, weights of incorrect dimension
        }

        // Weights are stored in Row major format
        for (unsigned int i = 0; i < ACADO_NY; i++) {
            for (unsigned int j = 0; j < ACADO_NY; j++) {
                ACADO.W[ACADO_NY*i + j] = 0;
            }
        }

        for (unsigned int i = 0; i < ACADO_NYN; i++) {
            for (unsigned int j = 0; j < ACADO_NYN; j++) {
                ACADO.WN[ACADO_NYN*i + j] = 0;
            }
        }

        for (unsigned int diag = 0; diag < ACADO_NY; diag++) {
			ACADO.W[ACADO_NY*diag + diag] = Wdiag[diag];
        }

        for (unsigned int diag = 0; diag < ACADO_NYN; diag++) {
			ACADO.WN[ACADO_NYN*diag + diag] = WNdiag[diag];
        }
    }

    void MPC::setReferences(void)
    {
        for (unsigned int i = 0; i < ACADO_N; i++) {
            ACADO.y[i].x_err = 0;
            ACADO.y[i].y_err = 0;
            ACADO.y[i].q2 = 0;
            ACADO.y[i].q3 = 0;
            ACADO.y[i].omega_ref_x = 0;
            ACADO.y[i].omega_ref_y = 0;
            ACADO.y[i].velocity_matching = 0;
            ACADO.y[i].velocity_error = 0;
            ACADO.y[i].domega_ref_x = 0;
            ACADO.y[i].domega_ref_y = 0;
            ACADO.y[i].velocity_slack = 0;
            ACADO.y[i].proximity_slack = 0;
        }
        ACADO.yN.x_err = 0;
        ACADO.yN.y_err = 0;
        ACADO.yN.q2 = 0;
        ACADO.yN.q3 = 0;
        ACADO.yN.omega_ref_x = 0;
        ACADO.yN.omega_ref_y = 0;
        ACADO.yN.velocity_matching = 0;
        ACADO.yN.velocity_error = 0;
    }

    double MPC::extractHeading(const float q[4])
    {
    	const float q_float[4] = { q[0], q[1], q[2], q[3] };
    	const float x_axis_body[3] = {1,0,0};
    	float x_axis_inertial[3];
    	Quaternion_RotateVector_Body2Inertial(q_float, x_axis_body, x_axis_inertial); // Phi(q)*Gamma(q)'*[0,1,0,0]'
        return atan2(x_axis_inertial[1], x_axis_inertial[0]); //  atan2(y,x);
    }

    /* Position and velocity is given in inertial frame */
    void MPC::setCurrentState(const double position[2], const double velocity[2], const double q[4])
    {
        position_[0] = position[0];
        position_[1] = position[1];
        velocity_[0] = velocity[0];
        velocity_[1] = velocity[1];
        quaternion_[0] = q[0];
        quaternion_[0] = q[0];
        quaternion_[0] = q[0];

        /* Extract current robot heading/yaw from attitude quaternion */
        const float q_float[4] = { q[0], q[1], q[2], q[3] };
        double RobotYaw = extractHeading(q_float);
        const float q_heading[4] = { cos(RobotYaw/2), 0, 0, sin(RobotYaw/2) }; // w,x,y,z
        float q_tilt[4];
        Quaternion_PhiT(q_heading, q_float, q_tilt); // q_heading' * q    (tilt defined in heading frame)
        if (q_tilt[0] < 0)
        	Quaternion_Conjugate(q_tilt); // invert q_tilt to take shortest rotation

        /* Compute velocity in window frame */
        double WindowVelocity[2];
        Math_Rotate2D(-WindowOrientation_, velocity, WindowVelocity);

        /* Compute approximated tilt quaternion in the window frame, for use in the linearly approximated MPC model
         * - where acceleration is defined in the heading frame independently by the q2 (x) and q3 (y) components of the tilt quaternion*/
        float q_tilt_inertial[4];
        Quaternion_GammaT(q_float, q_heading, q_tilt_inertial); // tilt defined in inertial frame
        const double q_tiltInertial_q2q3[2] = { q_tilt_inertial[1], q_tilt_inertial[2] };
        double q_tiltWindow_q2q3[2];
        Math_Rotate2D(-WindowOrientation_, q_tiltInertial_q2q3, q_tiltWindow_q2q3);

        /* Compute window position */
        const double WindowCenteredPosition[2] = { position[0] - windowPathOrigin_[0], position[1] - windowPathOrigin_[1] };
        double WindowPosition[2];
        Math_Rotate2D(-WindowOrientation_, WindowCenteredPosition, WindowPosition);

        /* Set ACADO initialization states */
        ACADO.x0.q2 = q_tiltWindow_q2q3[0]; // q2
        ACADO.x0.q3 = q_tiltWindow_q2q3[1]; // q3
        ACADO.x0.x = WindowPosition[0];
        ACADO.x0.y = WindowPosition[1];
        ACADO.x0.dx = WindowVelocity[0];
        ACADO.x0.dy = WindowVelocity[1];

        /* Should not modify path parameter, since this is kept static by the shifting strategy, as closestPositionOnCurrentPathToOrigin defines the starting point on the current path */
        ACADO.x0.s = ACADO.x[1].s; // set the initial guess to the shifted horizon value of the path parameter, s
        //ACADO.x[0].s = ACADO.x0.s = 0;
        //ACADO.x[0].ds = ACADO.x0.ds = 0;

        /* Set current angular velocity - this could also be set using the quaternion derivative. For now we just use the previously set value */
        ACADO.x0.omega_ref_x = controlBodyAngularVelocity_[0];
        ACADO.x0.omega_ref_y = controlBodyAngularVelocity_[1];
    }

    double MPC::getCurrentPathPosition() const
    {
        return ACADO.x0.s;
    }

    void MPC::resetStates(void)
    {
        ACADO.x0.q2 = 0;
        ACADO.x0.q3 = 0;
        ACADO.x0.x = 0;
        ACADO.x0.y = 0;
        ACADO.x0.dx = 0.001; // dx, dy and ds has to be initialized with a very small value apparently - otherwise the solver is infeasible from the get-go
        ACADO.x0.dy = 0.001;
        ACADO.x0.s = 0;
        ACADO.x0.ds = 0.001;
        ACADO.x0.omega_ref_x = 0;
        ACADO.x0.omega_ref_y = 0;

        /* Initialize horizon states to same values */
        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            ACADO.x[i].q2 = ACADO.x0.q2;
            ACADO.x[i].q3 = ACADO.x0.q3;
            ACADO.x[i].x = ACADO.x0.x;
            ACADO.x[i].y = ACADO.x0.y;
            ACADO.x[i].dx = ACADO.x0.dx;
            ACADO.x[i].dy = ACADO.x0.dy;
            ACADO.x[i].s = ACADO.x0.s;
            ACADO.x[i].ds = ACADO.x0.ds;
            ACADO.x[i].omega_ref_x = ACADO.x0.omega_ref_x;
            ACADO.x[i].omega_ref_y = ACADO.x0.omega_ref_y;
        }

        for (unsigned int i = 0; i < ACADO_N; i++) {
            ACADO.u[i].domega_ref_x = 0;
            ACADO.u[i].domega_ref_y = 0;
            ACADO.u[i].dds = 0;
            ACADO.u[i].velocity_slack = 0;
            ACADO.u[i].proximity_slack = 0;
        }

        controlBodyAngularVelocity_[0] = 0;
        controlBodyAngularVelocity_[1] = 0;
        controlBodyAngularVelocity_[2] = 0;
    }

    void MPC::shiftStates(void)
    {
        // Shift the states and controls by discarding the first step in the horizon (being the current step) and shifting all later steps one timestep forward.
        // Set the new final state (end state of horizon) to the same value as the shifted final state
        // OBS. This should be done as one of the first things before running the solver - by doing it here instead of after running the solver, the predicted horizon will be kept in the the ACADO variables until next step
        for (unsigned int i = 0; i < ACADO_N; i++) {
            ACADO.x[i].q2 = ACADO.x[i+1].q2;
            ACADO.x[i].q3 = ACADO.x[i+1].q3;
            ACADO.x[i].x = ACADO.x[i+1].x;
            ACADO.x[i].y = ACADO.x[i+1].y;
            ACADO.x[i].dx = ACADO.x[i+1].dx;
            ACADO.x[i].dy = ACADO.x[i+1].dy;
            ACADO.x[i].s = ACADO.x[i+1].s;
            ACADO.x[i].ds = ACADO.x[i+1].ds;
            ACADO.x[i].omega_ref_x = ACADO.x[i+1].omega_ref_x;
            ACADO.x[i].omega_ref_y = ACADO.x[i+1].omega_ref_y;
        }

        for (unsigned int i = 0; i < (ACADO_N-1); i++) {
            ACADO.u[i].domega_ref_x = ACADO.u[i+1].domega_ref_x;
            ACADO.u[i].domega_ref_y = ACADO.u[i+1].domega_ref_y;
            ACADO.u[i].dds = ACADO.u[i+1].dds;
            ACADO.u[i].velocity_slack = ACADO.u[i+1].velocity_slack;
            ACADO.u[i].proximity_slack = ACADO.u[i+1].proximity_slack;
        }

        ACADO.x[0].q2 = ACADO.x0.q2;
        ACADO.x[0].q3 = ACADO.x0.q3;
        ACADO.x[0].x = ACADO.x0.x;
        ACADO.x[0].y = ACADO.x0.y;
        ACADO.x[0].dx = ACADO.x0.dx;
        ACADO.x[0].dy = ACADO.x0.dy;
        ACADO.x[0].s = ACADO.x0.s;
        ACADO.x[0].ds = ACADO.x0.ds;
        ACADO.x[0].omega_ref_x = ACADO.x0.omega_ref_x;
        ACADO.x[0].omega_ref_y = ACADO.x0.omega_ref_y;
    }


    double MPC::getWindowAngularVelocityX(void) { return std::fmax(std::fmin(controlBodyAngularVelocity_[0], maxAngularVelocity_), -maxAngularVelocity_); }
    double MPC::getWindowAngularVelocityY(void) { return std::fmax(std::fmin(controlBodyAngularVelocity_[1], maxAngularVelocity_), -maxAngularVelocity_); }
    double MPC::getWindowAngularVelocityZ(void) { return std::fmax(std::fmin(controlBodyAngularVelocity_[2], maxAngularVelocity_), -maxAngularVelocity_); }

    void MPC::getInertialAngularVelocity(double angularVelocity[2])
    {
        // convert MPC W_omega_ref to K_omega_ref   (from window frame, which the MPC is working within, to inertial frame)
        const double H_omega_ref_2D[2] = { getWindowAngularVelocityX(), getWindowAngularVelocityY() }; // omega_ref in heading frame
        Math_Rotate2D(WindowOrientation_, H_omega_ref_2D, angularVelocity); // angularVelocity == I_omega_ref_2D
    }

    std::vector<std::pair<double,double>> MPC::getInertialAngularVelocityHorizon(void)
    {
        std::vector<std::pair<double,double>> angularVelocities;

        for (unsigned int i = (ACADO_N+1); i >= 1; i--) {
            const double H_omega_ref_2D[2] = { std::fmax(std::fmin(ACADO.x[i].omega_ref_x, maxAngularVelocity_), -maxAngularVelocity_),
                                         std::fmax(std::fmin(ACADO.x[i].omega_ref_y, maxAngularVelocity_), -maxAngularVelocity_) };
            double I_omega_ref_2D[2];
            Math_Rotate2D(WindowOrientation_, H_omega_ref_2D, I_omega_ref_2D);

            std::pair<double,double> angularVelocity;
            angularVelocity.first = I_omega_ref_2D[0];
            angularVelocity.second = I_omega_ref_2D[1];
            angularVelocities.push_back(angularVelocity);
        }

        return angularVelocities;
    }

    /*void MPC::EnableDebugOutput()
    {
        // See "QPOASES_PRINTLEVEL" flag in "acado_qpoases3_interface.h" - eg. set to PL_HIGH instead of PL_NONE
    }*/

    void MPC::resetACADO()
    {
        /* Reset ACADO workspace */
        // Either we can just initialize the solver cleanly
        acado_initializeSolver(); // basically does   memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
        // Or we can initialize it by forward simulation  // requires acadoVariables to be initialized
        //acado_initializeNodesByForwardSimulation();

        #if ACADO_USE_ARRIVAL_COST == 1
        acado_updateArrivalCost( 1 );   // what does this do? And does the argument need to be 1 or 0?
        #endif // ACADO_USE_ARRIVAL_COST == 1
    }

    void MPC::Step()
    {
        acado_timer t;          // to measure solvetime of solver call
        acado_tic(&t);

        /* Shift states - done here instead of after, to keep both the current and predicted MPC values in the ACADO variable */
        shiftStates();

        /* Prediction step */
        acado_preparationStep();

        /* Feedback step */
        SolverStatus_ = (status_t)acado_feedbackStep();

        /* Get solver status and debug info */
        SolverKKT_ = acado_getKKT();
        SolverCostValue_ = acado_getObjective();
        SolverIterations_ = acado_getNWSR();

        /* Shifting */
        #if ACADO_USE_ARRIVAL_COST == 1
        acado_updateArrivalCost( 0 );
        #endif // ACADO_USE_ARRIVAL_COST == 1

        //acado_shiftStates(strategy, xEnd, uEnd);
        // Shifting strategy: Move all samples forward, discarding the first/current element and 1) Initialize last node with xEnd or 2) Initialize node 51 by forward simulation.
        /*acado_shiftStates(1, 0, 0); // shift samples and keep the last value the same
        acado_shiftControls( 0 );*/

        /* return outputs and prepare next iteration */
        //for ( int i = 0; i < ACADO_NU; i++ ) out_u0[i] = acadoVariables.u[i];

        /*controlBodyAngularVelocity_[0] = ACADO.u[0].omega_ref_x;
        controlBodyAngularVelocity_[1] = ACADO.u[0].omega_ref_y;
        controlBodyAngularVelocity_[2] = 0;*/

        /* The MPC is controlling the angular acceleration, thus the current angular velocity output is taken as the state one step ahead
         * (gives the same as integrating the current domega_ref_x and domega_ref_y output from the MPC) */
        controlBodyAngularVelocity_[0] = ACADO.x[1].omega_ref_x;
        controlBodyAngularVelocity_[1] = ACADO.x[1].omega_ref_y;
        controlBodyAngularVelocity_[2] = 0;

        if ((ACADO.x[ACADO_N].s + closestPositionOnCurrentPathToOrigin_) > windowPathLength_) {
            Debug::printf("[WARN] MPC horizon is running outside of input path - Path length=%.2f vs s[N]=%.2f\n", windowPathLength_, (ACADO.x[ACADO_N].s + closestPositionOnCurrentPathToOrigin_));
        }

        /* calculate compute time */
        SolveTime_ = static_cast<float>(acado_toc(&t));

        Debug::printf("Solve time: %.2f\n", SolveTime_);
    }

    Trajectory MPC::getPredictedTrajectory(void)
    {
        Trajectory predictedTrajectory;

        for (unsigned int i = 0; i < (ACADO_N+1); i++) {
            double velocity = sqrt(ACADO.x[i].dx*ACADO.x[i].dx + ACADO.x[i].dy*ACADO.x[i].dy);
            //double velocity = ACADO.x[i].ds;
            predictedTrajectory.AddPoint(ACADO.x[i].x, ACADO.x[i].y, 0, velocity);
        }

        return predictedTrajectory;
    }

    MPC::state_t MPC::getHorizonState(unsigned int horizonIndex)
    {
        MPC::state_t state;
        if (horizonIndex >= (ACADO_N+1)) return state; // return an empty/zero state, since we are outside the horizon

        /* Rotate quaternion into inertial frame - the MPC quaternion elements are in window frame */
        float q_tilt_heading_q2q3[2] = { ACADO.x[horizonIndex].q2, ACADO.x[horizonIndex].q3 };
        float q_tilt_inertial_q2q3[2];
        Math_Rotate2D(WindowOrientation_, q_tilt_heading_q2q3, q_tilt_inertial_q2q3);

        float q1 = sqrt(1 - q_tilt_inertial_q2q3[0]*q_tilt_inertial_q2q3[0] - q_tilt_inertial_q2q3[1]*q_tilt_inertial_q2q3[1]);
        float q_tilt_inertial[4] = { q1, q_tilt_inertial_q2q3[0], q_tilt_inertial_q2q3[1], 0 };

        /* Compute heading quaternion */
        const float q_float[4] = { quaternion_[0], quaternion_[1], quaternion_[2], quaternion_[3] };
        float RobotYaw = extractHeading(q_float);
        float q_heading[4] = { cosf(RobotYaw/2), 0, 0, sinf(RobotYaw/2) }; // w,x,y,z

        /* Construct resulting quaternion by combining the tilt and heading */
        float q[4];
        Quaternion_Phi(q_tilt_inertial, q_heading, q); // q = q_tilt_inertial * q_heading   (tilt defined in inertial frame)

        /* Compute inertial frame position */
        double HorizonPosition[2] = { ACADO.x[horizonIndex].x, ACADO.x[horizonIndex].y };
        double InertialPosition[2];
        Math_Rotate2D(WindowOrientation_, HorizonPosition, InertialPosition);
        InertialPosition[0] += windowPathOrigin_[0];
        InertialPosition[1] += windowPathOrigin_[1];

        /* Rotate estimated velocity from window frame to inertial frame */
        double HorizonVelocity[2] = { ACADO.x[horizonIndex].dx, ACADO.x[horizonIndex].dy };
        double InertialVelocity[2];
        Math_Rotate2D(WindowOrientation_, HorizonVelocity, InertialVelocity);

        /* Prepare output variable */
        state.quaternion[0] = q[0];
        state.quaternion[1] = q[1];
        state.quaternion[2] = q[2];
        state.quaternion[3] = q[3];
        state.position[0] = InertialPosition[0];
        state.position[1] = InertialPosition[1];
        state.velocity[0] = InertialVelocity[0];
        state.velocity[1] = InertialVelocity[1];
        state.pathDistance = ACADO.x[horizonIndex].s;
        state.pathVelocity = ACADO.x[horizonIndex].ds;

        return state;
    }

    double MPC::getHorizonPathLength() const
    {
        return ACADO.x[ACADO_N].s;
    }

    MPC::status_t MPC::getStatus() const
    {
        return SolverStatus_;
    }

    double MPC::getSolveTime() const
    {
        return SolveTime_;
    }

    double MPC::getSampleTime() const
    {
        return ACADO_TS;
    }

}
