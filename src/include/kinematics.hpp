#include "math.hpp"


class inverseKinematics {
    double trackWidth;
    public:
    inverseKinematics(double itrackWidth):
        trackWidth(itrackWidth){};
    double toRightWheelSpeeds(double linearSpeed, double angularVelocity);
    double toLeftWheelSpeeds(double linearSpeed, double angularVelocity);
    double toRightWheelSpeeds2(double linearSpeed, double curvature);
    double toLeftWheelSpeeds2(double linearSpeed, double curvature);
};