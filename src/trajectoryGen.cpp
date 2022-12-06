#include "trajectoryGen.hpp"

TrajectoryGeneration::TrajectoryGeneration(KinematicConstraints constraints) : maxVel(constraints.maxVel),
                                                                               maxAccel(constraints.maxAccel),
                                                                               maxJerk(constraints.maxJerk){};

wpi::InterpolatingMap<double, double> TrajectoryGeneration::generateTrajectory(Vector2D istart, Vector2D iend)
{
    std::cout << "hihi \n";
    CubicBezier bezier(istart, iend);
    std::cout << "hihi \n";
    DescretePath path = bezier.generatePathByLength(0.03);
    std::cout << "hihi \n";
    imposeLimits(path);
    std::cout << "hihi \n";
    trajProfile[0].vel = 0;
    trajProfile.back().vel = 0;

    for (auto &thing : trajProfile)
    {
        std::cout << thing.vel << std::endl;
    }
    

    for (size_t i = 1; i < trajProfile.size(); i++)
    {
        std::cout << "hi bro \n";
        trajProfile[i].vel = std::min(trajProfile[i].vel, sqrt(trajProfile[i - 1].vel * trajProfile[i - 1].vel + 2 * maxAccel * path.getDeltaLength()));
    }
    std::cout << "hi2hi \n";
    for (size_t i = trajProfile.size() - 2; i >= 0; i--)
    {
        trajProfile[i].vel = std::min(trajProfile[i].vel, sqrt(trajProfile[i + 1].vel * trajProfile[i + 1].vel + 2 * maxAccel * path.getDeltaLength()));
    }
    std::cout << "hi3hi \n";
    for (size_t i = 0; i < trajProfile.size() - 1; i++)
    {
        trajProfile[i].accel = (trajProfile[i + 1].vel * trajProfile[i + 1].vel - trajProfile[i].vel * trajProfile[i].vel) / (2 * path.getDeltaLength());
    }
    std::cout << "hi4hi \n";
    for (size_t i = 1; i < trajProfile.size(); i++)
    {
        if (trajProfile[i - 1].accel != 0)
        {
            trajProfile[i].time = trajProfile[i - 1].time + (trajProfile[i].vel - trajProfile[i - 1].vel) / trajProfile[i - 1].accel;
        }
        else
        {
            trajProfile[i].time = trajProfile[i - 1].time + path.getDeltaLength() / trajProfile[i].vel;
        }
    }
std::cout << "hihi5 \n";
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
    return out;
}

double TrajectoryGeneration::getFinalTime()
{
    return finalTime;
}

void TrajectoryGeneration::imposeLimits(DescretePath &path)
{
    for (int i = 0; i < path.getSize(); i++)
    {
        Trajectory placeholder;
        placeholder.position = i * path.getDeltaLength();
        double maxAllowableVel = std::min(2 * maxVel / (2 + abs(path.getCurvature(i)) * trackWidth), maxVel);
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

