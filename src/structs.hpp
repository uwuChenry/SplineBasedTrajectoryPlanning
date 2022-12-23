#pragma once
#include "descretePath.hpp"
#include "interpolatingMap.hpp"

//vel accel pos time
struct Trajectory{
    double vel;
    double accel;
    double position;
    double time;
};

struct VelWithCurvature{
    double vel;
    double curvature;
    VelWithCurvature (double ivel, double icurvature):
    vel(ivel), curvature(icurvature){};
};

//vel accel position curvature
struct TrajectoryPoint{
    double vel;
    double accel;
    double position;
    double curvature;
};

struct PointWithCurvature{
    Point2D point;
    double curvature;
};

struct InterpolatingVelWithCurvature{
    wpi::InterpolatingMap<double, double> vel;
    wpi::InterpolatingMap<double, double> curvature;    
};


struct KinematicConstraints{
    double maxVel;
    double maxAccel;
    double maxJerk;

    KinematicConstraints(double imaxVel, double imaxAccel, double imaxJerk):
    maxVel(imaxVel), maxAccel(imaxAccel), maxJerk(imaxJerk){};
};