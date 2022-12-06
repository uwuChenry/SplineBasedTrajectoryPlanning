#include "main.h"
#include "bits/stdc++.h"


int main(){
    
    KinematicConstraints constraints(1.2, 2.3, 8);
    std::cout << "asdfasdfas1" << std::endl;
    TrajectoryGeneration generator(constraints);
    std::cout << "asdfasdfas2" << std::endl;
    auto mapp = generator.generateTrajectory({0, 0, 0},{2, 2, 0});
    std::cout << "asdfasdfas3" << std::endl;
    for (size_t i = 0; i < generator.getFinalTime(); i+= 0.01)
    {
        std::cout << mapp[i];
    }
    

    
}