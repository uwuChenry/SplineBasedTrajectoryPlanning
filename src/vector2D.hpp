#pragma once

#include "point2D.hpp"
#include <cmath>
#include "math.hpp"



class Vector2D{
    double x, y, angle, mag;

    public:
    Vector2D(double x, double y, double angle, double mag = 1)
    : x{x}
    , y{y}
    , angle{angle}
    , mag{mag}{}
    
    Point2D getSecondPoint();
    Point2D getPoint();
    void addAngle(double relativeAngle);
};
