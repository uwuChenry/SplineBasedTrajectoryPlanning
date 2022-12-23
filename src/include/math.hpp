#pragma once

#include <cmath>
#include "point2D.hpp"

class Math{
    public:
    //static double getCircumRadius(Point2D a, Point2D b, Point2D c);
    static double getCircumRadius2(Point2D a, Point2D b, Point2D c);
    //static bool isLeft(Point2D first, Point2D second, Point2D third);
    static constexpr double pi = 3.1415926535897932384626433;
};