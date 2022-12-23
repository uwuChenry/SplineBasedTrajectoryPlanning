#include "main.h"
#include "bits/stdc++.h"


int main(){
    /*
    std::vector<Trajectory> leftTraj;
    std::vector<Trajectory> rightTraj;
    KinematicConstraints constraints (1.2, 2.3, 8);
    inverseKinematics kinematics (0.3048);
    TrajectoryGeneration generator (constraints, 0.3048);
    auto thing = generator.generateTrajectory3({0, 0, 0}, {2, 2, 0});
    
    
    for (double i = 0; i < generator.getFinalTime(); i += 0.01){
        double vel = thing[i].vel;
        double accel = thing[i].accel;
        double position = thing[i].position;
        double curv = thing[i].curvature;
        

        
        leftTraj.push_back({
            kinematics.toLeftWheelSpeeds2(vel, curv),
            kinematics.toLeftWheelAccel2(accel, curv),
            kinematics.toLeftWheelSpeeds2(position, curv),
            i });
        rightTraj.push_back({
            kinematics.toRightWheelSpeeds2(vel, curv),
            kinematics.toRightWheelAccel2(accel, curv),
            kinematics.toRightWheelSpeeds2(position, curv),
            i });
    }

    for (auto thing : leftTraj){
        std::cout << thing.time << std::endl;
    }*/
    
    std::vector<Trajectory> leftTraj;
    std::vector<Trajectory> rightTraj;
    KinematicConstraints constraints(1.2, 2.3, 8);
    inverseKinematics kinematics (0.3048);
    TrajectoryGeneration generator(constraints, 0.3048);
    auto thing = generator.generateTrajectory2({0, 0, 0}, {2,2,0});
    std::cout << generator.getFinalTime() << "final time\n";
    scurveProfile scurve (constraints);
    CubicBezier bezier ({0, 0, 0}, {2,2,0});
    scurve.generateProfile(bezier.getLength());
    for (auto& thing : scurve.pathTrajectory){
        std::cout << thing.vel << std::endl;
    }

    //rightTraj.front().position = 0.0;
    //leftTraj.front().position = 0.0;


    std::cout << "hi";
    for (double i = 0; i < generator.getFinalTime(); i+= 0.01){
        //std::cout << thing.curvature[i] << std::endl;
        double vel = thing.vel[i];
        double curv = thing.curvature[i];
        //std::cout << thing.vel[i] << "\n";
        //std::cout << " [" << thing.vel[i] << " , " << thing.curvature[i] << "] \n";
        //std::cout << vel * curv << std::endl;
        Trajectory leftPlaceholder;
        leftPlaceholder.vel = kinematics.toLeftWheelSpeeds2(vel, curv);
        leftPlaceholder.time = i;
        leftTraj.push_back(leftPlaceholder);
        Trajectory rightPlaceholder;
        rightPlaceholder.vel = kinematics.toRightWheelSpeeds2(vel, curv);
        rightPlaceholder.time = i;
        rightTraj.push_back(rightPlaceholder);

    }

    //std::cout << leftTraj[0].position << "pos 1 \n";
    leftTraj[0].position = 0;
    rightTraj[0].position = 0;

    for (size_t i = 0; i < leftTraj.size() - 1; i++){
        leftTraj[i].accel = (leftTraj[i+1].vel - leftTraj[i].vel) / (leftTraj[i+1].time - leftTraj[i].time);
        rightTraj[i].accel = (rightTraj[i+1].vel - rightTraj[i].vel) / (rightTraj[i+1].time - rightTraj[i].time);
        //std::cout << rightTraj[i+1].vel << "    ";
    }
    //leftTraj.back().accel = leftTraj[leftTraj.size() - 2].accel;
    //rightTraj.back().accel = rightTraj[rightTraj.size() - 2].accel;
    leftTraj.back().accel = 0;
    rightTraj.back().accel = 0;

    for (size_t i = 1; i < leftTraj.size(); i++){
        leftTraj[i].position = leftTraj[i - 1].position + leftTraj[i].vel * 0.01 + leftTraj[i].accel * 0.01 * 0.01 / 2.0;
        rightTraj[i].position = rightTraj[i - 1].position + rightTraj[i].vel * 0.01 + leftTraj[i].accel * 0.01 * 0.01 / 2.0;
    }

    leftTraj.pop_back();
    rightTraj.pop_back();

    for (auto thing : rightTraj){
        //std::cout << thing.accel << std::endl;
    }
    







    /*auto mapp = generator.generateTrajectory({0, 0, 0, 0.5},{2, 2, 0, 0.5});
    std::cout << generator.getFinalTime() << std::endl;
    for (double i = 0; i < generator.getFinalTime(); i += 0.01)
    {
        std::cout << mapp[i] << " ";
    }
    std::cout << "end \n";

    CubicBezier bezier ({0,0,0},{2,2,0});*/
    /*for (double i = 0; i <= 1; i += 0.01){
        std::cout << bezier.getCurvature(i) << "    ";
    }
    std::cout << "test";*/
 
    
}