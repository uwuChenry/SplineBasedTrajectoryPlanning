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