#include "include/trajectoryGen.hpp"

TrajectoryGeneration::TrajectoryGeneration(KinematicConstraints constraints, double trackWidth) 
    : maxVel(constraints.maxVel)
    , maxAccel(constraints.maxAccel)
    , maxJerk(constraints.maxJerk)
    , trackWidth(trackWidth){
    };



double TrajectoryGeneration::getFinalTime(){
    return finalTime;
}


//imposelimits
void TrajectoryGeneration::imposeLimits(DescretePathWithCurvature &path){
    for (int i = 0; i < path.getSize(); i++){
        Trajectory placeholder;
        placeholder.position = i * path.getDeltaLength();
        double maxAllowableVel = std::min(2.0 * maxVel / (2.0 + fabs(path.getCurvature(i)) * trackWidth), maxVel);
        placeholder.vel = maxAllowableVel;
        trajProfile.push_back(placeholder);
    }
}

void TrajectoryGeneration::imposeLimitsPositionVel(std::vector<VelocityLimit> ilimits, DescretePathWithCurvature &path){
    for(size_t i = 0; i < ilimits.size(); i++){
        for(double p = ilimits[i].dStart; p <= ilimits[i].dEnd; p += path.getDeltaLength()){
            trajProfile[p/path.getDeltaLength()].vel = ilimits[i].velocity;
        }
    }
}



InterpolatingVelWithCurvature TrajectoryGeneration::generateTrajectory(Vector2D istart, Vector2D iend){
    CubicBezier bezier (istart, iend);
    DescretePathWithCurvature path = bezier.generatePathByLengthWithCurvature(0.01, 2000);

    imposeLimits(path);
    trajProfile[0].vel = 0;
    trajProfile.back().vel = 0;
    trajProfile[0].time = 0;
    
    wpi::InterpolatingMap<double, double> outVel;
    wpi::InterpolatingMap<double, double> outCurvature;
    
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


    for (size_t i = 0; i < trajProfile.size(); i++){
        outVel.insert(trajProfile[i].time, trajProfile[i].vel);
        outCurvature.insert(trajProfile[i].time, path.curvature[i]);
    }
    
    finalTime = trajProfile.back().time;

    return {outVel, outCurvature};
}

    