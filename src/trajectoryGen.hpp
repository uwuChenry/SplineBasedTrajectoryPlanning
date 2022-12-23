#pragma once

#include "descretePath.hpp"
#include <vector>
#include "structs.hpp"
#include "interpolatingMap.hpp"
#include <iostream>
#include "vector2D.hpp"
#include "bezier.hpp"
#include "interpolatingMap2.hpp"




class TrajectoryGeneration{
    double maxVel, maxAccel, maxJerk;
    double trackWidth;
    double finalTime;  

    DescretePath path;
    std::vector<Trajectory> trajProfile;
    std::vector<std::pair<Trajectory, Trajectory>> generatedPath;

    public:

    TrajectoryGeneration(KinematicConstraints constraints, double trackWidth);

    wpi::InterpolatingMap<double, double> generateTrajectory(Vector2D istart, Vector2D iend);
    InterpolatingVelWithCurvature generateTrajectory2(Vector2D istart, Vector2D iend);
    InterpolatingTrajectoryPoint generateTrajectory3(Vector2D istart, Vector2D iend);

    double getFinalTime();

    void imposeLimits(DescretePath& path);

    void imposeLimits2(DescretePath &path);

    void imposeLimits3(DescretePathWithCurvature &path);
    
    void printTrajectoryProfile(TrajectoryGetMode igetStuffMode);
    
    
    std::vector<std::pair<Trajectory, Trajectory>> calculateFinalTrajectoryProfile();



};