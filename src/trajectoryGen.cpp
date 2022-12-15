#include "trajectoryGen.hpp"

TrajectoryGeneration::TrajectoryGeneration(KinematicConstraints constraints, double trackWidth) 
    : maxVel(constraints.maxVel)
    , maxAccel(constraints.maxAccel)
    , maxJerk(constraints.maxJerk)
    , trackWidth(trackWidth){};

wpi::InterpolatingMap<double, double> TrajectoryGeneration::generateTrajectory(Vector2D istart, Vector2D iend)
{
    CubicBezier bezier (istart, iend);
    //std::cout<<bezier.getLength(1000)<<" bezier length from gen traj\n";
    DescretePath path = bezier.generatePathByLength(0.01);

    //path.setDeltaLength(path.getDeltaLength());
    imposeLimits2(path);
    trajProfile[0].vel = 0;
    trajProfile.back().vel = 0;
    trajProfile[0].time = 0;
    
    std::cout << "gonig into gen traj \n";
    //std::cout << trajProfile[0].vel;
    for (size_t i = 1; i < trajProfile.size(); i++)
    {
        //std::cout<<path.getDeltaLength();
        //std::cout<< trajProfile[i].vel<< "   " << (sqrt(trajProfile[i - 1].vel * trajProfile[i - 1].vel + 2 * maxAccel * path.getDeltaLength())) << "\n";
        //std::cout << std::min(trajProfile[i].vel, (sqrt(trajProfile[i - 1].vel * trajProfile[i - 1].vel + 2 * maxAccel * path.getDeltaLength()))) << "      ";
        trajProfile[i].vel = std::min(trajProfile[i].vel, (sqrt(trajProfile[i - 1].vel * trajProfile[i - 1].vel + 2 * maxAccel * path.getDeltaLength())));
        
    }
    for (int i = trajProfile.size() - 2; i >= 0; i--)
    {
        //std::cout<<trajProfile[i].vel<<std::endl;
        trajProfile[i].vel = std::min(trajProfile[i].vel, sqrt(trajProfile[i + 1].vel * trajProfile[i + 1].vel + 2 * maxAccel * path.getDeltaLength()));
        
    }
    for (size_t i = 0; i < trajProfile.size() - 1; i++)
    {
        trajProfile[i].accel = (trajProfile[i + 1].vel * trajProfile[i + 1].vel - trajProfile[i].vel * trajProfile[i].vel) / (2 * path.getDeltaLength());
        //std::cout<< trajProfile[i].accel << "    ";
    }
    for (size_t i = 1; i < trajProfile.size(); i++)
    {
        //std::cout<<trajProfile[i-1].time;
        if (trajProfile[i - 1].accel != 0)
        {
            trajProfile[i].time = trajProfile[i - 1].time + (trajProfile[i].vel - trajProfile[i - 1].vel) / trajProfile[i - 1].accel;
        }
        else
        {
            trajProfile[i].time = trajProfile[i - 1].time + path.getDeltaLength() / trajProfile[i].vel;
        }
        //std::cout << trajProfile[i].vel << "     ";
        //std::cout<< trajProfile[i].time << "    ";
    }
    std::cout << "finish for loop \n";
    wpi::InterpolatingMap<double, double> out;
    for (size_t i = 0; i < trajProfile.size(); i++)
    {
        /*TrajectoryPoint ret;
        ret.vel = trajProfile[i].vel;
        ret.accel = trajProfile[i].accel;
        ret.position = trajProfile[i].position;*/
        out.insert(trajProfile[i].time, trajProfile[i].vel);
    }
    finalTime = trajProfile.back().time;
    std::cout << trajProfile.back().time << " final time \n";
    std::cout << "finished genTraj" <<std::endl;
    for (size_t i = 0; i < trajProfile.size(); i++)
    {
        std::cout<< trajProfile[i].position << " pos     ";
    }
    
    return out;
}


double TrajectoryGeneration::getFinalTime()
{
    return finalTime;
}

void TrajectoryGeneration::imposeLimits2(DescretePath &path){
    for (int i = 0; i < path.getSize(); i++)
    {
        Trajectory placeholder;
        placeholder.position = i * path.getDeltaLength();
        double left = maxVel - trackWidth / 2.0 * (maxVel * fabs(path.getCurvature(i))) / Math::pi;
        double right = maxVel + trackWidth / 2.0 * (maxVel * fabs(path.getCurvature(i))) / Math::pi;
        double realMaxSpeed = std::max(fabs(left), fabs(right));
        if (realMaxSpeed > maxVel){
            left = left / realMaxSpeed * maxVel;
            right = right / realMaxSpeed * maxVel;
        }
        double maxAllowableVel = (left + right) / 2.0;
        std::cout << maxAllowableVel << " vel     ";
        //double maxAllowableVel = std::min(2.0 * maxVel / (2.0 + abs(path.getCurvature(i)) * trackWidth), maxVel);
        //double thing = 2 * maxVel / (2 + abs(path.getCurvature(i)) * trackWidth);
        //std::cout << trackWidth << "trackwidth     ";
        //std::cout << thing << " calc max vel     ";
        //std::cout << path.getCurvature(i) << " curvature     ";
        //std::cout << (2.0 + fabs(path.getCurvature(i)) * trackWidth) << " oo aa      ";
        placeholder.vel = maxAllowableVel;
        trajProfile.push_back(placeholder);
    }
    std::cout << trajProfile.size() << "trajprofile size";
}


void TrajectoryGeneration::imposeLimits(DescretePath &path)
{
    //std::cout << "impose limit thing"<< path.getDeltaLength();
    for (int i = 0; i < path.getSize(); i++)
    {
        Trajectory placeholder;
        placeholder.position = i * path.getDeltaLength();
        double maxAllowableVel = std::min(2.0 * maxVel / (2.0 + abs(path.getCurvature(i)) * trackWidth), maxVel);
        double thing = 2 * maxVel / (2 + abs(path.getCurvature(i)) * trackWidth);
        //std::cout << trackWidth << "trackwidth     ";
        //std::cout << thing << " calc max vel     ";
        //std::cout << path.getCurvature(i) << " curvature     ";
        std::cout << (2.0 + fabs(path.getCurvature(i)) * trackWidth) << " oo aa      ";
        placeholder.vel = maxAllowableVel;
        trajProfile.push_back(placeholder);
    }
    std::cout << trajProfile.size() << "trajprofile size";
}



void TrajectoryGeneration::printTrajectoryProfile(TrajectoryGetMode igetStuffMode){
    TrajectoryGetMode mode = igetStuffMode;
    for (size_t i = 0; i < trajProfile.size(); i++)
    {
        if (mode == TrajectoryGetMode::vel)
            std::cout << trajProfile[i].vel;
        if (mode == TrajectoryGetMode::accel)
            std::cout << trajProfile[i].accel;
        if (mode == TrajectoryGetMode::pos)
            std::cout << trajProfile[i].position;
    }
}

