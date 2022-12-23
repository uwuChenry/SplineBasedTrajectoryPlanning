#pragma once

#include "descretePath.hpp"
#include <vector>
#include "structs.hpp"
#include "interpolatingMap.hpp"
#include <iostream>
#include "vector2D.hpp"
#include "bezier.hpp"
//#include "interpolatingMap2.hpp"




class TrajectoryGeneration{
    double maxVel, maxAccel, maxJerk;
    double trackWidth;
    double finalTime;  

    std::vector<Trajectory> trajProfile;
    std::vector<std::pair<Trajectory, Trajectory>> generatedPath;

    public:

    TrajectoryGeneration(KinematicConstraints constraints, double trackWidth);

    InterpolatingVelWithCurvature generateTrajectory(Vector2D istart, Vector2D iend);

    double getFinalTime();

    void imposeLimits(DescretePathWithCurvature &path);

    void imposeLimitsPositionVel(std::vector<VelocityLimit> ilimits, DescretePathWithCurvature &path);
    
    
    std::vector<std::pair<Trajectory, Trajectory>> calculateFinalTrajectoryProfile();



};