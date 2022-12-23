#include "scurve.hpp"
#include "structs.hpp"
#include "iostream"



Trajectory scurveProfile::calculateTrajectory(double time)
{
    if (time < timePhase[0]){
        double pos1 = jerk * time * time * time / 6;
        return {(jerk * time * time / 2), (jerk * time), pos1, time};
    }
    else if (time < timePhase[1]){
        double dt2 = time - timePhase[0];
        double pos2 = fPosPhase[0] + fVelPhase[0] * dt2 + aPeak * dt2 * dt2 / 2;
        return {(fVelPhase[0] + (aPeak * dt2)), (aPeak), pos2, time};
    }
    else if (time < timePhase[2]){
        double dt3 = time - timePhase[1];
        double pos3 = fPosPhase[1] + fVelPhase[1] * dt3 + aPeak * dt3 * dt3 / 2 - jerk * dt3 * dt3 * dt3 / 6;
        return {(fVelPhase[1] + (aPeak * dt3) - (jerk * dt3 * dt3 / 2)), (aPeak - (jerk * dt3)), pos3, time};
    }
    else if (time < timePhase[3]){
        double dt4 = time - timePhase[2];
        double pos4 = fPosPhase[2] + fVelPhase[2] * dt4;
        return {(fVelPhase[2]), 0, pos4, time};
    }
    else if (time < timePhase[4]){
        double dt5 = time - timePhase[3];
        double pos5 = fPosPhase[3] + fVelPhase[3] * dt5 - jerk * dt5 * dt5 * dt5 / 6;
        return {(fVelPhase[3] - (jerk * dt5 * dt5 / 2)), (-jerk * dt5), pos5, time};
    }
    else if (time < timePhase[5]){
        double dt6 = time - timePhase[4];
        double pos6 = fPosPhase[4] + fVelPhase[4] * dt6 - aPeak * dt6 * dt6 / 2;
        return {(fVelPhase[4] - (aPeak * dt6)), (-aPeak), pos6, time};
    }
    else if (time < timePhase[6]){
        double dt7 = time - timePhase[5];
        double pos7 = fPosPhase[5] + fVelPhase[5] * dt7 - aPeak * dt7 * dt7 / 2 + jerk * dt7 * dt7 * dt7 / 6;
        return {(fVelPhase[5] + (-aPeak * dt7) + (jerk * dt7 * dt7 / 2)), (-aPeak + jerk * dt7), pos7, time};
    }
    else{
        return {0, 0, 0, 0};
    }
}

void scurveProfile::generateProfile(double idistance){
    double distance = idistance;
    double taa = aMax / jerk;

    double velInStageThreeStart = vMax - (jerk * taa * taa / 2);
    double velInStageOneEnd = jerk * taa * taa / 2;
    double ta = (velInStageThreeStart - velInStageOneEnd) / aMax;
    double totalAccelTime = taa + taa + ta;
    double distanceInA = totalAccelTime * vMax;
    double tc = (distance - distanceInA) / vMax;
    bool isP4;

    double x = cbrt(((4 * distance * distance) / jerk) / distance);
    if (x < (aMax * 2 / jerk) && x >= 0) isP4 = true;
    
    if (tc > 0){ //7 phase
        std::cout << "7 phase" << std::endl;
        timePhase[0] = taa;
        timePhase[1] = taa + ta;
        timePhase[2] = totalAccelTime;
        timePhase[3] = tc + totalAccelTime;
        timePhase[4] = timePhase[3] + taa;
        timePhase[5] = timePhase[3] + taa + ta;
        timePhase[6] = timePhase[3] + totalAccelTime;
    }   

    if (tc <= 0 && isP4 == false){ //6 phase
        double dist = (distance / 2) - (jerk * taa * taa * taa);
        double a = (jerk * taa / 2);
        double b = ((jerk * taa * taa / 2) + (taa * taa * jerk));
        double c = -dist;
        double root1 = (-b + sqrt((b * b) - (4 * a * c))) / (2 * a);
        double root2 = (-b - sqrt((b * b) - (4 * a * c))) / (2 * a);
        double ta6;
        if (root1 > 0){
            ta6 = root1;
        }
        else if (root2 > 0){
            ta6 = root2;
        }

        std::cout << "6 phase" << std::endl;
        timePhase[0] = taa;
        timePhase[1] = ta6 + taa;
        timePhase[2] = taa + taa + ta6;
        timePhase[3] = timePhase[2];
        timePhase[4] = timePhase[2] + taa;
        timePhase[5] = timePhase[2] + ta6 + taa;
        timePhase[6] = timePhase[2] + taa + taa + ta6;
    }

    if (tc <= 0 && isP4 == true){ //4 phase
        double taa4 = cbrt((distance / 2) / jerk);
        std::cout << "4 Phase" << std::endl;
        timePhase[0] = taa4;
        timePhase[1] = taa4;
        timePhase[2] = 2 * taa4;
        timePhase[3] = 2 * taa4;
        timePhase[4] = 3 * taa4;
        timePhase[5] = 3 * taa4;
        timePhase[6] = 4 * taa4;
    }

    //final time for each segments
    fTimePhase[0] = timePhase[0];
    fTimePhase[1] = timePhase[1] - timePhase[0];
    fTimePhase[2] = timePhase[2] - timePhase[1];
    fTimePhase[3] = timePhase[3] - timePhase[2];
    fTimePhase[4] = timePhase[4] - timePhase[3];
    fTimePhase[5] = timePhase[5] - timePhase[4];
    fTimePhase[6] = timePhase[6] - timePhase[5];

    //peak accel
    aPeak = jerk * timePhase[0];

    //final velocity
    fVelPhase[0] = (jerk * fTimePhase[0] * fTimePhase[0] / 2);
    fVelPhase[1] = (fVelPhase[0] + (jerk * fTimePhase[0] * fTimePhase[1]));
    fVelPhase[2] = (fVelPhase[1] + (aPeak * fTimePhase[2]) - (jerk * fTimePhase[2] * fTimePhase[2] / 2));
    fVelPhase[3] = fVelPhase[2];
    fVelPhase[4] = fVelPhase[3] - (jerk * fTimePhase[4] * fTimePhase[4] / 2);
    fVelPhase[5] = fVelPhase[4] - (jerk * fTimePhase[4] * fTimePhase[5]);
    fVelPhase[6] = fVelPhase[5] - (jerk * fTimePhase[6]) + (jerk * fTimePhase[6] * fTimePhase[6] / 2);

    //final position for each time period
    fPosPhase[0] = jerk * fTimePhase[0] * fTimePhase[0] * fTimePhase[0] / 6;
    fPosPhase[1] = fPosPhase[0] + fVelPhase[0] * fTimePhase[1] + aPeak * fTimePhase[1] * fTimePhase[1] / 2;
    fPosPhase[2] = fPosPhase[1] + fVelPhase[1] * fTimePhase[2] + aPeak * fTimePhase[2] * fTimePhase[2] / 2 - jerk * fTimePhase[2] * fTimePhase[2] * fTimePhase[2] / 6;
    fPosPhase[3] = fPosPhase[2] + fVelPhase[2] * fTimePhase[3];
    fPosPhase[4] = fPosPhase[3] + fVelPhase[3] * fTimePhase[4] - jerk * fTimePhase[4] * fTimePhase[4] * fTimePhase[4] / 6;
    fPosPhase[5] = fPosPhase[4] + fVelPhase[4] * fTimePhase[5] - aPeak * fTimePhase[5] * fTimePhase[5] / 2;
    fPosPhase[6] = fPosPhase[5] + fVelPhase[5] * fTimePhase[6] - aPeak * fTimePhase[6] * fTimePhase[6] / 2 + jerk * fTimePhase[6] * fTimePhase[6] * fTimePhase[6] / 6;


    int stepAmount = timePhase[6] * 100 - 5;
    for (int i = 0; i < stepAmount; i++){
        double currentTime = i * 10;
        double currentTimeInS = currentTime / 1000;
        auto traj = calculateTrajectory(currentTimeInS);
        pathTrajectory.push_back(traj);
    }
}





