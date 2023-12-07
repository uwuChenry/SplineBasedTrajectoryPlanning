#include "include/main.h"
#include <iostream>


int closestPointIndex(Point2D point, std::vector<Point2D>& path){
        double smallestDist = 1.79769e+308;
        int a = 0;
        for (int i = 0; i < (int)path.size(); i++){
            double dist = Point2D::distanceBetween(point, path[i]);
            if (dist <= smallestDist){
                smallestDist = dist;
                a = i;
            } 
        }
    return a;
}



int main(){   
    // std::vector<Trajectory> leftTraj;
    // std::vector<Trajectory> rightTraj;

    // KinematicConstraints constraints(1.45, 2.55, 6);
    // inverseKinematics kinematics (0.3048);
    // TrajectoryGeneration generator(constraints, 0.3048);
    // auto thing = generator.generateTrajectory({0, 0, 0}, {2,0,0});

    // for (double i = 0; i < generator.getFinalTime(); i+= 0.01){
    //     double vel = thing.vel[i];
    //     double curv = thing.curvature[i];
    //     Trajectory leftPlaceholder;
    //     leftPlaceholder.vel = kinematics.toLeftWheelSpeeds2(vel, curv);
    //     leftPlaceholder.time = i;
    //     leftTraj.push_back(leftPlaceholder);
    //     Trajectory rightPlaceholder;
    //     rightPlaceholder.vel = kinematics.toRightWheelSpeeds2(vel, curv);
    //     rightPlaceholder.time = i;
    //     rightTraj.push_back(rightPlaceholder);

    // }

    // leftTraj[0].position = 0;
    // rightTraj[0].position = 0;

    // for (size_t i = 0; i < leftTraj.size() - 1; i++){
    //     leftTraj[i].accel = (leftTraj[i+1].vel - leftTraj[i].vel) / (leftTraj[i+1].time - leftTraj[i].time);
    //     rightTraj[i].accel = (rightTraj[i+1].vel - rightTraj[i].vel) / (rightTraj[i+1].time - rightTraj[i].time);
    // }

    // leftTraj.back().accel = 0;
    // rightTraj.back().accel = 0;

    // for (size_t i = 1; i < leftTraj.size(); i++){
    //     leftTraj[i].position = leftTraj[i - 1].position + leftTraj[i].vel * 0.01 + leftTraj[i].accel * 0.01 * 0.01 / 2.0;
    //     rightTraj[i].position = rightTraj[i - 1].position + rightTraj[i].vel * 0.01 + leftTraj[i].accel * 0.01 * 0.01 / 2.0;
    // }

    // leftTraj.pop_back();
    // rightTraj.pop_back();






    // CubicBezier bezier ({0, 0},{0, 1},{0,2},{1,2});
    // CubicBezier bezier ({0, 0, 90, 2}, {1,1.5, -90});
    // DescretePathWithCurvature path = bezier.generatePathByLengthWithCurvature(0.01, 2000);
    // for (int i = 0; i < path.getSize(); i++)
    // {
    //     std::cout << path.path[i].getX() << std::endl;
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;
    // std::cout << std::endl;
    // for (int i = 0; i < path.getSize(); i++)
    // {
    //     std::cout << path.path[i].getY() << std::endl;
    // }

    Point2D start (1, 1);
    std::vector<Point2D> points = {{0.0, 0.0}, {1.2, 1.2}, {2,2}, {1.1, 1.1}, {1.05, 1.05}};

    std::cout << closestPointIndex(start, points) << std::endl;
}