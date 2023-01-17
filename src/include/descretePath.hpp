#pragma once

#include <vector>
#include "point2D.hpp"
#include <cmath>
#include "math.hpp"
#include "structs.hpp"

class DescretePathWithCurvature{

    double deltaLength;
    double distance;
    
    public:
    DescretePathWithCurvature(){};
    std::vector<Point2D> path = {};
    std::vector<double> curvature = {};

    void setDistance(double idistance);
    
    double getDistance();
    
    void setDeltaLength(double ilength);
    
    double getDeltaLength();
    
    int getSize();

    double getCurvature(int index);

};