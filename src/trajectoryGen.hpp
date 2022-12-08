#pragma once

#include "descretePath.hpp"
#include <vector>
#include "structs.hpp"
#include "interpolatingMap.hpp"
#include <iostream>
#include "vector2D.hpp"
#include "bezier.hpp"




class TrajectoryGeneration{
    double maxVel, maxAccel, maxJerk;
    double trackWidth;
    double finalTime;  

    DescretePath path;
    CubicBezier bezier;
    std::vector<Trajectory> trajProfile;
    std::vector<std::pair<Trajectory, Trajectory>> generatedPath;

    public:

    TrajectoryGeneration(KinematicConstraints constraints, double trackWidth);

    wpi::InterpolatingMap<double, double> generateTrajectory(Vector2D istart, Vector2D iend);

    double getFinalTime();

    void imposeLimits(DescretePath& path);
    
    void printTrajectoryProfile(TrajectoryGetMode igetStuffMode);
    
    
    std::vector<std::pair<Trajectory, Trajectory>> calculateFinalTrajectoryProfile();



};