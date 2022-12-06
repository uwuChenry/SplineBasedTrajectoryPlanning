#pragma once

#include <vector>
#include "point2D.hpp"
#include <cmath>
#include "math.hpp"


class DescretePath{
    std::vector<Point2D> path;
    double deltaLength;
    double distance;
    
    public:
    DescretePath(){};

    void setDistance(double idistance);
    
    double getDistance();
    
    void setDeltaLength(double ilength);
    
    double getDeltaLength();
    
    void pushBack(Point2D point);
    
    void popBack();
    
    Point2D back();
    
    int getSize();

    double getCurvature(int index);

};