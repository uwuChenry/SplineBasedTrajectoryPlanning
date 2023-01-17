#pragma once

#include "descretePath.hpp"
#include <vector>
#include "structs.hpp"
#include "interpolatingMap.hpp"
#include <iostream>
#include "vector2D.hpp"
#include "bezier.hpp"
#include "scurve.hpp"
//#include "interpolatingMap2.hpp"




class TrajectoryGeneration{
    double maxVel, maxAccel, maxJerk;
    double trackWidth;
    double finalTime; 

    std::vector<Trajectory> trajProfile;

    public:

    TrajectoryGeneration(KinematicConstraints constraints, double trackWidth);


    //trapezoidal
    InterpolatingVelWithCurvature generateTrajectory(Vector2D istart, Vector2D iend);

    //scurve
    InterpolatingVelWithCurvature generateTrajectory2(Vector2D istart, Vector2D iend);

    double getFinalTime();

    //trapezoidal
    void imposeLimits(DescretePathWithCurvature &path);

    //scurve
    void imposeLimits2(DescretePathWithCurvature &path, scurveProfile &scurve, CubicBezier &bezier);

    //custom pos vel limit
    void imposeLimitsPositionVel(std::vector<VelocityLimit> ilimits, DescretePathWithCurvature &path);

};