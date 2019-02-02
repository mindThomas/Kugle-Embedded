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
 
#ifndef LIBRARY_PATH_H
#define LIBRARY_PATH_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <cstdint>

#include <vector>

#include "Trajectory.h"

namespace MPC
{

    class Polynomial
    {
        public:
            Polynomial();
            Polynomial(const Polynomial& poly);
            Polynomial(unsigned int order);
            Polynomial(std::vector<double> coeffs);
            Polynomial(const double * coeffs, int num_coeffs);
            ~Polynomial();

            Polynomial& operator=(const Polynomial& other);
            Polynomial operator+(const Polynomial& other) const;
            Polynomial operator-(const Polynomial& other) const;
            Polynomial operator+(const double& offset) const;
            Polynomial operator-(const double& offset) const;

            void print();

            double evaluate(double t);
            double operator()(double t);
            std::vector<double> evaluate(std::vector<double> tVec);
            std::vector<double> operator()(std::vector<double> tVec);

            int order() const;
            Polynomial squared() const;
            Polynomial derivative() const;

            double findMinimum(double s_init, double s_lower, double s_upper, double stoppingCriteria, unsigned int maxIterations);

            double getCoefficient(unsigned int index);

        //private:
            void FitPoints(unsigned int order, std::vector<double>& tVec, std::vector<double>& values, bool EnforceBeginEndConstraint, bool EnforceBeginEndAngleConstraint);
            //Eigen::VectorXd ConstrainedLeastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq, double lambda);

        private:
            std::vector<double> coeffs_;
    };

    class Path
    {
        public:
            Path();
            Path(Polynomial& poly_x, Polynomial& poly_y, double s_end = -1);
            Path(Trajectory& trajectory, unsigned int approximationOrder = 6, bool StopAtEnd = false, bool EnforceBeginEndConstraint = true, bool EnforceBeginEndAngleConstraint = true);
            ~Path();

            Path& operator=(const Path& other);

            Point get(double s);
            Point operator()(double s);

            std::vector<Point> get(std::vector<double> sVec);
            std::vector<Point> operator()(std::vector<double> sVec);

            void print();

            int order();

            double getXcoefficient(unsigned int index);
            double getYcoefficient(unsigned int index);
            double length();

            double FindClosestPoint(const Point& position);

        private:
            double ArcCurveLength(double t);
            double ApproximateArcCurveLength(double t, double discretizationStepSize = 0.0001);
            void FitTrajectory(Trajectory& trajectory, unsigned int approximationOrder = 6, bool StopAtEnd = false, bool EnforceBeginEndConstraint = true, bool EnforceBeginEndAngleConstraint = true);

        private:
            /* The path polynomials should be parameterized by arc curve length, s ! */
            Polynomial poly_x_;
            Polynomial poly_y_;
            double s_end_;

            bool computed_dsquared_;
            Polynomial dsquared_;
    };

}
	
	
#endif
