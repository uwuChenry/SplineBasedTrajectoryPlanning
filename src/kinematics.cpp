#include "include/kinematics.hpp"


double inverseKinematics::toRightWheelSpeeds(double linearSpeed, double angularVelocity){
    return (linearSpeed + trackWidth / 2 * angularVelocity);
}
double inverseKinematics::toLeftWheelSpeeds(double linearSpeed, double angularVelocity){
    return (linearSpeed - trackWidth / 2 * angularVelocity);
}

double inverseKinematics::toRightWheelSpeeds2(double linearSpeed, double curvature){
    return linearSpeed * (2 + curvature * trackWidth) / 2;
}
double inverseKinematics::toLeftWheelSpeeds2(double linearSpeed, double curvature){
    return linearSpeed * (2 - curvature * trackWidth) / 2;
}

