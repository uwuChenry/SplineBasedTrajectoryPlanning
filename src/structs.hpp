#pragma once
#include "descretePath.hpp"
#include "interpolatingMap.hpp"


struct Trajectory{
    double vel;
    double accel;
    double position;
    double time;
};

struct TrajectoryPoint{
    double vel;
    double accel;
    double position;
};

struct PointWithCurvature{
    Point2D point;
    double curvature;
};

struct InterpolatingVelWithCurvature{
    wpi::InterpolatingMap<double, double> vel;
    wpi::InterpolatingMap<double, double> curvature;    
};

enum class TrajectoryGetMode{
    vel = 0,
    accel = 1,
    pos = 2,
};


struct KinematicConstraints{
    double maxVel;
    double maxAccel;
    double maxJerk;

    KinematicConstraints(double imaxVel, double imaxAccel, double imaxJerk):
    maxVel(imaxVel), maxAccel(imaxAccel), maxJerk(imaxJerk){};
};