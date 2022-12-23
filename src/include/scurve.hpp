#pragma once
#include <string>
#include <vector>
#include <map>
#include "structs.hpp"
#include <optional>
#include <atomic>


class scurveProfile{
    private:

    KinematicConstraints constraints;

    double vMax = 1.5; //1.5
    double aMax = 2.5; //2.3
    double jerk = 15;  //5
    double timePhase[7];
    double fTimePhase[7];
    double fVelPhase[7];
    double fPosPhase[7];

    //peak accel
    double aPeak;

    //void generateTime(double idistance);
    Trajectory calculateTrajectory(double time);
    //Trajectory calculateTrajectorySimple(double time);
    //void calculateTrajectoryLinear2(double distance, bool isReversed);
    public:
    void generateProfile(double idistance);
    //void generateProfileSimple(double idistance);
 
    std::vector<VelocityLimit> generateVelocityLimits(double idistance);

    scurveProfile (
        KinematicConstraints iconstraints)
        : constraints(iconstraints)
        , vMax(iconstraints.maxVel)
        , aMax(iconstraints.maxAccel)
        , jerk(iconstraints.maxJerk)
        {}
    std::vector<Trajectory> pathTrajectory;
};
