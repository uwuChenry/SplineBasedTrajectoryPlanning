#include "main.h"
#include "bits/stdc++.h"


int main(){
    
    KinematicConstraints constraints(1.2, 2.3, 8);
    TrajectoryGeneration generator(constraints, 0.3048);
    auto mapp = generator.generateTrajectory({0, 0, 0, 0.5},{2, 2, 0, 0.5});
    /*std::cout << generator.getFinalTime() << std::endl;
    for (double i = 0; i < generator.getFinalTime(); i += 0.01)
    {
        //std::cout << i <<std::endl;
        //std::cout << mapp[i];
    }
    std::cout << "end";*/
    CubicBezier bezier({0, 0}, {0, 0.5}, {1.5, 2}, {2, 2});
    std::cout<<bezier.getLength()<< std::endl;

    CubicBezier bezier2({0, 0, 0, 0.5}, {2, 2, 0, 0.5});
    std::cout<<bezier.getLength();
    
}