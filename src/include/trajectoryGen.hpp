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
    //scurveProfile scurve; 

    std::vector<Trajectory> trajProfile;
    //std::vector<std::pair<Trajectory, Trajectory>> generatedPath;

    public:

    TrajectoryGeneration(KinematicConstraints constraints, double trackWidth);

    InterpolatingVelWithCurvature generateTrajectory(Vector2D istart, Vector2D iend);
    InterpolatingVelWithCurvature generateTrajectory2(Vector2D istart, Vector2D iend);

    double getFinalTime();

    void imposeLimits(DescretePathWithCurvature &path);
    void imposeLimits2(DescretePathWithCurvature &path, scurveProfile &scurve, CubicBezier &bezier);
    void imposeLimitsPositionVel(std::vector<VelocityLimit> ilimits, DescretePathWithCurvature &path);

    
    //std::vector<std::pair<Trajectory, Trajectory>> calculateFinalTrajectoryProfile();



};