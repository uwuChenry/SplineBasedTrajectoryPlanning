#include "trajectoryGen.hpp"

TrajectoryGeneration::TrajectoryGeneration(KinematicConstraints constraints, double trackWidth) 
    : maxVel(constraints.maxVel)
    , maxAccel(constraints.maxAccel)
    , maxJerk(constraints.maxJerk)
    , trackWidth(trackWidth){};

wpi::InterpolatingMap<double, double> TrajectoryGeneration::generateTrajectory(Vector2D istart, Vector2D iend){
    CubicBezier bezier (istart, iend);
    DescretePath path = bezier.generatePathByLength(0.01);

    imposeLimits(path);
    trajProfile[0].vel = 0;
    trajProfile.back().vel = 0;
    trajProfile[0].time = 0;
    
    //std::cout << "gonig into gen traj \n";
    for (size_t i = 1; i < trajProfile.size(); i++){
        trajProfile[i].vel = std::min(trajProfile[i].vel, (sqrt(trajProfile[i - 1].vel * trajProfile[i - 1].vel + 2 * maxAccel * path.getDeltaLength())));
    }

    for (int i = trajProfile.size() - 2; i >= 0; i--){
        trajProfile[i].vel = std::min(trajProfile[i].vel, sqrt(trajProfile[i + 1].vel * trajProfile[i + 1].vel + 2 * maxAccel * path.getDeltaLength()));    
    }
    
    for (size_t i = 0; i < trajProfile.size() - 1; i++){
        trajProfile[i].accel = (trajProfile[i + 1].vel * trajProfile[i + 1].vel - trajProfile[i].vel * trajProfile[i].vel) / (2 * path.getDeltaLength());
    }

    for (size_t i = 1; i < trajProfile.size(); i++){
        if (trajProfile[i - 1].accel != 0){
            trajProfile[i].time = trajProfile[i - 1].time + (trajProfile[i].vel - trajProfile[i - 1].vel) / trajProfile[i - 1].accel;
        }
        else{
            trajProfile[i].time = trajProfile[i - 1].time + path.getDeltaLength() / trajProfile[i].vel;
        }
    }
    //std::cout << "finish for loop \n";
    wpi::InterpolatingMap<double, double> out;
    for (size_t i = 0; i < trajProfile.size(); i++)
    {
        out.insert(trajProfile[i].time, trajProfile[i].vel);
    }
    finalTime = trajProfile.back().time;
    //std::cout << trajProfile.back().time << " final time \n";
    //std::cout << "finished genTraj" <<std::endl;
    //for (size_t i = 0; i < trajProfile.size(); i++)
    //{
    //    std::cout<< trajProfile[i].position << " pos     ";
    //}
    
    return out;
}


InterpolatingVelWithCurvature TrajectoryGeneration::generateTrajectory2(Vector2D istart, Vector2D iend){
    CubicBezier bezier (istart, iend);
    //DescretePath path = bezier.generatePathByLength(0.01);
    DescretePathWithCurvature path = bezier.generatePathByLengthWithCurvature(0.01, 2000);

    imposeLimits3(path);
    trajProfile[0].vel = 0;
    trajProfile.back().vel = 0;
    trajProfile[0].time = 0;
    
    for (size_t i = 1; i < trajProfile.size(); i++){
        trajProfile[i].vel = std::min(trajProfile[i].vel, (sqrt(trajProfile[i - 1].vel * trajProfile[i - 1].vel + 2 * maxAccel * path.getDeltaLength())));
    }

    for (int i = trajProfile.size() - 2; i >= 0; i--){
        trajProfile[i].vel = std::min(trajProfile[i].vel, sqrt(trajProfile[i + 1].vel * trajProfile[i + 1].vel + 2 * maxAccel * path.getDeltaLength()));    
    }
    
    for (size_t i = 0; i < trajProfile.size() - 1; i++){
        trajProfile[i].accel = (trajProfile[i + 1].vel * trajProfile[i + 1].vel - trajProfile[i].vel * trajProfile[i].vel) / (2 * path.getDeltaLength());
    }

    for (size_t i = 1; i < trajProfile.size(); i++){
        if (trajProfile[i - 1].accel != 0){
            trajProfile[i].time = trajProfile[i - 1].time + (trajProfile[i].vel - trajProfile[i - 1].vel) / trajProfile[i - 1].accel;
        }
        else{
            trajProfile[i].time = trajProfile[i - 1].time + path.getDeltaLength() / trajProfile[i].vel;
        }
    }

    wpi::InterpolatingMap<double, double> outVel;
    wpi::InterpolatingMap<double, double> outCurvature;

    for (size_t i = 0; i < trajProfile.size(); i++){
        outVel.insert(trajProfile[i].time, trajProfile[i].vel);
    }
    for (size_t i = 0; i < trajProfile.size(); i++){
        outCurvature.insert(trajProfile[i].time, path.curvature[i]);
    }
    finalTime = trajProfile.back().time;



    for (auto thing : path.path){
        std::cout << thing.getX() << std::endl;
    }
    std::cout << "\n\n\n\n\n";
    for (auto thing : path.path){
        std::cout << thing.getY() << std::endl;
    }
    return {outVel, outCurvature};
}


double TrajectoryGeneration::getFinalTime()
{
    return finalTime;
}

//wpi method of calculating max vel
void TrajectoryGeneration::imposeLimits2(DescretePath &path){
    for (int i = 0; i < path.getSize(); i++){
        Trajectory placeholder;
        placeholder.position = i * path.getDeltaLength();
        double left = maxVel - trackWidth / 2.0 * (maxVel * fabs(path.getCurvature(i)));
        double right = maxVel + trackWidth / 2.0 * (maxVel * fabs(path.getCurvature(i)));
        double realMaxSpeed = std::max(fabs(left), fabs(right));
        if (realMaxSpeed > maxVel){
            left = left / realMaxSpeed * maxVel;
            right = right / realMaxSpeed * maxVel;
        }
        double maxAllowableVel = (left + right) / 2.0;
        placeholder.vel = maxAllowableVel;
        trajProfile.push_back(placeholder);
    }
    //std::cout << trajProfile.size() << "trajprofile size";
}

//simple way of calculating max vel from curvature
void TrajectoryGeneration::imposeLimits(DescretePath &path){
    for (int i = 0; i < path.getSize(); i++){
        Trajectory placeholder;
        placeholder.position = i * path.getDeltaLength();
        double maxAllowableVel = std::min(2.0 * maxVel / (2.0 + fabs(path.getCurvature(i)) * trackWidth), maxVel);
        //std::cout << maxAllowableVel << "vel    ";
        placeholder.vel = maxAllowableVel;
        trajProfile.push_back(placeholder);
    }
    //std::cout << trajProfile.size() << "trajprofile size";
}

//imposelimits
void TrajectoryGeneration::imposeLimits3(DescretePathWithCurvature &path){
        for (int i = 0; i < path.getSize(); i++){
        Trajectory placeholder;
        placeholder.position = i * path.getDeltaLength();
        
        double maxAllowableVel = std::min(2.0 * maxVel / (2.0 + fabs(path.getCurvature(i)) * trackWidth), maxVel);
        //std::cout << maxAllowableVel << "vel    ";
        placeholder.vel = maxAllowableVel;
        trajProfile.push_back(placeholder);
    }
}