#include "kinematics.hpp"


double inverseKinematics::toRightWheelSpeeds(double linearSpeed, double angularVelocity){
    return (linearSpeed + trackWidth / 2 * angularVelocity);
}
double inverseKinematics::toLeftWheelSpeeds(double linearSpeed, double angularVelocity){
    return (linearSpeed - trackWidth / 2 * angularVelocity);
}

//vel + width / 2 * vel * curv 


//vel * (2 + curv * width) / 2
//vel * (1 + (curv * width/2))
//vel + vel(curv * width /2)
//vel + vel * width / 2 * curv
//vel + width / 2 * vel * curv

//max = speed * (2+curv * track )/2
//2max = speed * (2 + curv * track)
//2max / (2 + curv * track) = speed

double inverseKinematics::toRightWheelSpeeds2(double linearSpeed, double curvature){
    return linearSpeed * (2 + curvature * trackWidth) / 2;
}
double inverseKinematics::toLeftWheelSpeeds2(double linearSpeed, double curvature){
    return linearSpeed * (2 - curvature * trackWidth) / 2;
}

double inverseKinematics::toRightWheelAccel2(double linearAccel, double curvature){
    return linearAccel * (2 + curvature * trackWidth) / 2;
}
double inverseKinematics::toLeftWheelAccel2(double linearAccel, double curvature){
    return linearAccel * (2 - curvature * trackWidth) / 2;
}

