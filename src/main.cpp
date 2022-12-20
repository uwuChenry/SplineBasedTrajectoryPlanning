#include "main.h"
#include "bits/stdc++.h"


int main(){
    
    KinematicConstraints constraints(1.2, 2.3, 8);
    TrajectoryGeneration generator(constraints, 0.3048);
    auto mapp = generator.generateTrajectory({0, 0, 0, 0.5},{2, 2, 0, 0.5});
    std::cout << generator.getFinalTime() << std::endl;
    for (double i = 0; i < generator.getFinalTime(); i += 0.01)
    {
        std::cout << mapp[i] << " ";
    }
    std::cout << "end \n";

    CubicBezier bezier ({0,0,0},{2,2,0});
    for (double i = 0; i <= 1; i += 0.01){
        std::cout << bezier.getCurvature(i) << "    ";
    }
    //std::cout << Math::getCircumRadius2({0, 0}, {0, 4}, {3, 0}) << " oo aa \n";
    //std::cout << Math::getCircumRadius({0, 0}, {0, 4}, {3, 0}) << " thing \n";

    
}