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

        // Internal things to kep track of

        // Task trampoline and task code
    //static void trampoline(void* instance);
    //void taskLoop();
    
    //void executePath();
    //void executePlannerPath();
    //void setTarget(QLength idistance, bool iisReversed);
    

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
    //void calculateTrajectoryLinear2(double distance, bool isReversed);
    
    void generateProfile(double idistance);
 
    //rpm to meterPerSecond
    double rpmToLinear(double rpm);
    double encoderTickToMeter(double encoderTicks);


    public:
    //constraints, chassis controller, scales, gear ratio, scurve ff left, scurve ff right, pp ff left, pp ff right
    scurveProfile (
        KinematicConstraints iconstraints)
        : constraints(iconstraints)
        , vMax(iconstraints.maxVel)
        , aMax(iconstraints.maxAccel)
        , jerk(iconstraints.maxJerk)
        {}
    std::vector<Trajectory> pathTrajectory;
};
