
#pragma once

#include "point2D.hpp"
#include "vector2D.hpp"
#include "descretePath.hpp"
#include <iostream>




class CubicBezier
{
    Point2D p1, c1, c2, p2;

    public:
    CubicBezier(Point2D start, Point2D control1, Point2D control2, Point2D end):
    p1(start),c1(control1),c2(control2),p2(start){};
    CubicBezier(Vector2D start, Vector2D end){
        end.addAngle(180);
        CubicBezier(start.getPoint(), start.getSecondPoint(), end.getSecondPoint(), end.getPoint());
    }

    double getLength(int step = 250);
    DescretePath generatePathByStep (int step = 500);
    DescretePath generatePathByLength (int length, int initDistStep = 250, int traverseStep = 1000, bool end = true);

    Point2D getPoint(double t) const;

    Point2D getVelocity(double t) const;
    Point2D getAcceleration(double t) const;
};

