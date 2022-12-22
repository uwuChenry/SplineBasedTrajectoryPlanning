#include "main.h"
#include "bits/stdc++.h"


int main(){
    std::vector<VelWithCurvature> pathWithLinearVelAndCurvature;
    std::vector<Trajectory> leftTraj;
    std::vector<Trajectory> rightTraj;
    KinematicConstraints constraints(1.2, 2.3, 8);
    inverseKinematics kinematics (0.3048);
    TrajectoryGeneration generator(constraints, 0.3048);
    auto thing = generator.generateTrajectory2({0, 0, 0}, {2,2,0});
    std::cout << generator.getFinalTime() << "final time\n";
    for (double i = 0; i < generator.getFinalTime(); i+= 0.02){
        //std::cout << thing.curvature[i] << std::endl;
        double vel = thing.vel[i];
        double curv = thing.curvature[i];
        //std::cout << thing.vel[i] << "\n";
        //std::cout << " [" << thing.vel[i] << " , " << thing.curvature[i] << "] \n";
        //std::cout << vel * curv << std::endl;
        pathWithLinearVelAndCurvature.push_back({vel, curv});
        Trajectory leftPlaceholder {kinematics.toLeftWheelSpeeds2(vel, curv), 0, 0, 0};
        Trajectory rightPlaceholder {kinematics.toRightWheelSpeeds2(vel, curv), 0, 0, 0};
        leftTraj.push_back(leftPlaceholder);
        rightTraj.push_back(rightPlaceholder);
    }
    for (auto thing : leftTraj){
        //std::cout << thing.vel << std::endl;
    }
    /*auto mapp = generator.generateTrajectory({0, 0, 0, 0.5},{2, 2, 0, 0.5});
    std::cout << generator.getFinalTime() << std::endl;
    for (double i = 0; i < generator.getFinalTime(); i += 0.01)
    {
        std::cout << mapp[i] << " ";
    }
    std::cout << "end \n";

    CubicBezier bezier ({0,0,0},{2,2,0});
    /*for (double i = 0; i <= 1; i += 0.01){
        std::cout << bezier.getCurvature(i) << "    ";
    }
    std::cout << "test";*/
 
    
}