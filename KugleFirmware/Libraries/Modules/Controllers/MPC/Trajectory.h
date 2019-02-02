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
 
#ifndef LIBRARY_TAJECTORY_H
#define LIBRARY_TAJECTORY_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <cstdint>

#include <vector>
#include <limits>


namespace MPC
{

	typedef struct Point
	{
		double x;
		double y;
		Point() : x(0), y(0) {};
		Point(double x_, double y_) : x(x_), y(y_) {};

		const Point operator+(const Point& other) const;
		const Point operator-(const Point& other) const;
		const Point operator-() const;
		const Point operator*(const double& scale) const;
		const Point operator/(const double& scale) const;
		Point& operator+=(const Point& other);
		Point& operator-=(const Point& other);
		Point& operator*=(const double& scale);
		Point& operator/=(const double& scale);
		void rotate(const double rotationAngle);

	};

	class TrajectoryPoint
	{
	    public:
            int seq; // sequence number
            Point point;
            bool goal; // is this point the goal/final point - hence the robot should stop here
            double heading;
            double velocity;

            TrajectoryPoint() : seq(-1), point(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()), goal(false), heading(std::numeric_limits<double>::infinity()), velocity(std::numeric_limits<double>::infinity()) {}; // uninitialized point
            TrajectoryPoint(int seq_, double _x, double _y, bool goal = false, double _heading = -1, double _velocity = -1) : seq(seq_), point(_x, _y), goal(goal), heading(_heading), velocity(_velocity) {};
            double EuclideanDistance(const TrajectoryPoint& other);
            double EuclideanDistance(const Point& other);

            /* Operator used for sorting */
            bool operator < (const TrajectoryPoint& p) const
            {
                return (seq < p.seq);
            }
	};

	class Trajectory
	{
		public:
            Trajectory();
            Trajectory(std::vector<TrajectoryPoint>& points);
            Trajectory(TrajectoryPoint points[], int num_points);
			~Trajectory();

            Trajectory& operator=(const Trajectory& other);

			void AddPoint(TrajectoryPoint& point);
            void AddPoint(Point point, bool goal = false, double heading = -1, double velocity = -1);
			void AddPoint(double x, double y, bool goal = false, double heading = -1, double velocity = -1);
            void AddPoint(int seq, double x, double y, bool goal = false, double heading = -1, double velocity = -1);
            void clear();
			size_t size();

            void rotate(Trajectory& rotatedTrajectory, double angle);
            void rotate(double angle);
            void move(double x, double y);
            void move(Point offset);
            void move(Trajectory& movedTrajectory, double x, double y);
            void scale(double scale_factor);
            void PositionExtract(Trajectory& extractedTrajectory, Point p_min, Point p_max);
            void PositionExtract(Trajectory& extractedTrajectory, double x_min, double y_min, double x_max, double y_max);
            void SequenceExtract(Trajectory& extractedTrajectory, int seq_min, int seq_max = std::numeric_limits<int>::infinity());
            void DistanceExtract(Trajectory& extractedTrajectory, double distance_max, double distance_min = 0);
            void ApproxCurveLengthExtract(Trajectory& extractedTrajectory, double curve_length_max);
			void WindowExtract(Trajectory& extractedTrajectory, Point center, double heading, double width, double height, Point offset = Point(0,0));
            void print();
            bool find(int seq, TrajectoryPoint& foundPoint);
			bool find(int seq);
            void sort();
            double distance();
            bool includesGoal();
            int GetLastSequenceID();
            std::vector<double> GetDistanceList();
			std::vector<double> GetX();
			std::vector<double> GetY();
            TrajectoryPoint get(unsigned int index);
            TrajectoryPoint back();

            void FixSequenceOrder(Trajectory& correctedTrajectory, int startSeqID, int endSeqID);
            void ExtractContinuousClosestSequence(Trajectory& continuousSequenceTrajectory, Point position = Point(0,0));
			TrajectoryPoint FindClosestPoint(Point position);

	    public:
	        static Trajectory GenerateTestTrajectory(void);

		private:
			std::vector<TrajectoryPoint> points_;
			int lastSeq_;
			bool sorted_;
	};

}
	
	
#endif
