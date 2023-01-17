#include "include/main.h"
#include "bits/stdc++.h"


int main(){   
    std::vector<Trajectory> leftTraj;
    std::vector<Trajectory> rightTraj;
    KinematicConstraints constraints(1.45, 2.55, 6);
    inverseKinematics kinematics (0.3048);
    TrajectoryGeneration generator(constraints, 0.3048);
    auto thing = generator.generateTrajectory2({0, 0, 0}, {2,2,0});

    for (double i = 0; i < generator.getFinalTime(); i+= 0.01){
        double vel = thing.vel[i];
        std::cout << vel << std::endl;
        double curv = thing.curvature[i];
        Trajectory leftPlaceholder;
        leftPlaceholder.vel = kinematics.toLeftWheelSpeeds2(vel, curv);
        leftPlaceholder.time = i;
        leftTraj.push_back(leftPlaceholder);
        Trajectory rightPlaceholder;
        rightPlaceholder.vel = kinematics.toRightWheelSpeeds2(vel, curv);
        rightPlaceholder.time = i;
        rightTraj.push_back(rightPlaceholder);

    }

    leftTraj[0].position = 0;
    rightTraj[0].position = 0;

    for (size_t i = 0; i < leftTraj.size() - 1; i++){
        leftTraj[i].accel = (leftTraj[i+1].vel - leftTraj[i].vel) / (leftTraj[i+1].time - leftTraj[i].time);
        rightTraj[i].accel = (rightTraj[i+1].vel - rightTraj[i].vel) / (rightTraj[i+1].time - rightTraj[i].time);
    }

    leftTraj.back().accel = 0;
    rightTraj.back().accel = 0;

    for (size_t i = 1; i < leftTraj.size(); i++){
        leftTraj[i].position = leftTraj[i - 1].position + leftTraj[i].vel * 0.01 + leftTraj[i].accel * 0.01 * 0.01 / 2.0;
        rightTraj[i].position = rightTraj[i - 1].position + rightTraj[i].vel * 0.01 + leftTraj[i].accel * 0.01 * 0.01 / 2.0;
    }

    leftTraj.pop_back();
    rightTraj.pop_back();

    for (auto thing : leftTraj){
        //std::cout << thing.vel << std::endl;
    }

    
    scurveProfile scurve (constraints);
    CubicBezier bezier({0, 0, 0}, {2, 0, 0});
    //scurve.generateProfileWithoutVector(bezier.getLength());
    std::cout << "hi \n";
    scurve.generateProfile(bezier.getLength());
    for (auto& thing : scurve.pathTrajectory){
        //std::cout << thing.vel << std::endl;
    }
    //std::cout << "\n\n\n\n\n\n";
    //std::cout << "a";
    for (double i = 0; i <= bezier.getLength(); i += 0.01){
        //std::cout << scurve.calculateTrajectoryFromDistance(i).vel << std::endl;
        
    }
}