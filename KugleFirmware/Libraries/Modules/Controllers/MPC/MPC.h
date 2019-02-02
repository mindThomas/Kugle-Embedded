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
 
#ifndef LIBRARY_MPC_H
#define LIBRARY_MPC_H

#include <cmath>

#define deg2rad(x) (M_PI*x/180.f)
#define rad2deg(x) (180.f*x/M_PI)

#include <stdlib.h>
#include <limits>
#include <vector>

#include "acado_common.h"
#include "acado_indices.h"

#include "Path.h"
#include "Trajectory.h"

namespace MPC {

    static float inf = std::numeric_limits<float>::infinity();

class Obstacle
{
    public:
        double x;
        double y;
        double radius;

        Obstacle() : x(999), y(999), radius(0.01) {}; // neglible obstacle
        Obstacle(double x_, double y_, double radius_) : x(x_), y(y_), radius(radius_) {};

        double proximity(const double position[2]) const
        {
            return sqrt( (x-position[0])*(x-position[0]) + (y-position[1])*(y-position[1]) ) - radius;
        }

        /* Operator used for sorting */
        static bool proximitySorting(const double position[2], const Obstacle& obstacle1, const Obstacle& obstacle2)
        {
            return (obstacle1.proximity(position) < obstacle2.proximity(position));
        }
};

class MPC
{
	public:
		static const unsigned int HorizonLength = ACADO_N;		// 'WNmat' needs to be a double matrix of size [ACADO_NY x ACADO_NY]
        static constexpr double RobotRadius = 0.1; // radius of robot in meters

		// Weight definitions
        static const double WPathFollow;
        static const double WVelocity;
        static const double WSmoothness;
        static const double Wdiag[ACADO_NY]; // Horizon weight matrix (cost)
        static const double WNdiag[ACADO_NYN]; // Final state weight matrix (cost)

	private:
		static ACADO_t& ACADO;
		static constexpr unsigned int num_StateVariables = ACADO_NX;	// 'xInit' needs to be a double vector of size [ACADO_NX x 1]
		static constexpr unsigned int num_Inputs = ACADO_NU;		    // 'uInit' needs to be a double vector of size [ACADO_NU x 1]
		static constexpr unsigned int num_Outputs = ACADO_NY;		    // 'Wmat' needs to be a double matrix of size [ACADO_NY x ACADO_NY]
		static constexpr unsigned int num_FinalOutputs = ACADO_NYN;		// 'WNmat' needs to be a double matrix of size [ACADO_NYN x ACADO_NYN]
		static constexpr unsigned int num_OnlineData = ACADO_NOD;		// 'odInit' needs to be a double matrix of size [ACADO_N+1 x ACADO_NOD]
                                                                        // 'refInit' needs to be a double matrix of size [ACADO_N+1 x ACADO_NY]

    public:
        typedef struct state_t
        {
            double quaternion[4];
            double position[2];
            double velocity[2];
            double pathDistance;
            double pathVelocity;
        } state_t;

	    typedef enum orientation_selection_t : uint8_t
        {
	        INERTIAL_FRAME = 0,
	        HEADING_FRAME = 1,
	        VELOCITY_FRAME = 2
        } orientation_selection_t;

	    typedef enum status_t
        {
            SUCCESS = 0,
            ITERATION_LIMIT_REACHED = 1,
            INFEASIBLE = -2,
            UNBOUNDED = -3,
			OTHER = -1
        } status_t;

	public:
		MPC();
		~MPC();

		void Reset();
		void Step();

        void setPath(Path& path, const double origin[2], const double currentPosition[2]);
        void setXYreferencePosition(const double position[2]);
        void setObstacles(std::vector<Obstacle>& obstacles);
		void setVelocityBounds(double min_velocity, double max_velocity);
		void setDesiredVelocity(double velocity);

        double getWindowAngularVelocityX(void);
        double getWindowAngularVelocityY(void);
        double getWindowAngularVelocityZ(void);
        void getInertialAngularVelocity(double angularVelocity[2]);
        std::vector<std::pair<double,double>> getInertialAngularVelocityHorizon(void);

		void setCurrentState(const double position[2], const double velocity[2], const double q[4]);
        void setControlLimits(double maxAngularVelocity, double maxAngle);
        void setWeightsDiag(const double * Wdiag, unsigned int numWdiag, const double * WNdiag, unsigned int numWNdiag);

        Trajectory getPredictedTrajectory(void);
        state_t getHorizonState(unsigned int horizonIndex = 1);
        double getHorizonPathLength() const;

        Path getCurrentPath(void);
		double getClosestPointOnPath(void);

		double extractHeading(const float q[4]);

		double getSampleTime() const;
		double getSolveTime() const;
        double getCurrentPathPosition() const;
        status_t getStatus() const;

    private:
        void resetACADO();
        void resetStates(void);
        void setReferences(void);
        void shiftStates(void);

	private:
        std::vector<Obstacle> currentObstacles_;

        double desiredVelocity_;
        double minVelocity_;
        double maxVelocity_;
        double maxAngle_;
        double maxAngularVelocity_;

        double controlBodyAngularVelocity_[2]; // this is the computed output of the MPC

        double WindowWidth_;
        double WindowHeight_;
        double WindowOffset_[2];
        double WindowOrientation_; // heading/yaw of the path window (in inertial frame)
        Path windowPath_;
        double windowPathLength_;
        double windowPathOrigin_[2]; // origin of path in inertial frame - hence what (0,0) of path corresponds to in inertial coordinates
        double closestPositionOnCurrentPathToOrigin_; // this corresponds to the initialization value of the 's' parameter - however this is actually input as OnlineData instead
		unsigned int pathApproximationOrder_;
        orientation_selection_t WindowOrientationSelection_;

		/* Both position and velocity is defined in inertial frame */
        double position_[2];
        double velocity_[2];
		double quaternion_[4];

		/* Solver status and outputs */
		status_t SolverStatus_;
		double SolverKKT_;
		double SolverCostValue_;
		int SolverIterations_;
		double SolveTime_;
    };
	
}
	
#endif
