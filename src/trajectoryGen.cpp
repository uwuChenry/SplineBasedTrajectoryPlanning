#include "trajectoryGen.hpp"

TrajectoryGeneration::TrajectoryGeneration(KinematicConstraints constraints, double trackWidth) 
    : maxVel(constraints.maxVel)
    , maxAccel(constraints.maxAccel)
    , maxJerk(constraints.maxJerk)
    , trackWidth(trackWidth){};

wpi::InterpolatingMap<double, double> TrajectoryGeneration::generateTrajectory(Vector2D istart, Vector2D iend)
{
    CubicBezier bezierPath(istart, iend);
    DescretePath path = bezierPath.generatePathByLength(0.03);
    std::cout<< bezierPath.getLength() << "length bezier \n";
    path.setDeltaLength(0.03);
    std::cout<<path.getSize() << "size path";
    imposeLimits(path);
    trajProfile[0].vel = 0;
    trajProfile.back().vel = 0;
    trajProfile[0].time = 0;

    /*for (auto &thing : trajProfile)
    {
        std::cout << thing.vel << std::endl;
    }*/
    
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
    std::cout << trajProfile.back().time << std::endl;
    std::cout << "finished genTraj" <<std::endl;
    return out;
}


double TrajectoryGeneration::getFinalTime()
{
    return finalTime;
}

void TrajectoryGeneration::imposeLimits(DescretePath &path)
{
    //std::cout << "impose limit thing"<< path.getDeltaLength();
    for (int i = 0; i < path.getSize(); i++)
    {
        Trajectory placeholder;
        placeholder.position = i * path.getDeltaLength();
        double maxAllowableVel = std::min(2 * maxVel / (2 + abs(path.getCurvature(i)) * trackWidth), maxVel);
        //std::cout << maxAllowableVel << std::endl;
        placeholder.vel = maxAllowableVel;
        trajProfile.push_back(placeholder);
    }
}

void TrajectoryGeneration::printTrajectoryProfile(TrajectoryGetMode igetStuffMode)
{
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

