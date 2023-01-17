
#pragma once

#include "point2D.hpp"
#include "vector2D.hpp"
#include "structs.hpp"
#include "descretePath.hpp"
#include <iostream>




class CubicBezier
{
    Point2D p1, c1, c2, p2;
    

    public:
    CubicBezier(Point2D start, Point2D control1, Point2D control2, Point2D end):
    p1(start),c1(control1),c2(control2),p2(end){};
    
    CubicBezier(Vector2D start, Vector2D end){
        end.addAngle(180);
        this->p1 = start.getPoint();
        this->c1 = start.getSecondPoint();
        this->c2 = end.getSecondPoint();
        this->p2 = end.getPoint();
    }
    CubicBezier(){
        CubicBezier({0, 0, 0}, {0, 0, 0});
    }
    void setPoints(Point2D start, Point2D control1, Point2D control2, Point2D end);
    void setPoints(Vector2D start, Vector2D end);
    double getLength(int step = 250);
    double getCurvature(double t) const;

    DescretePathWithCurvature generatePathByStep (int step = 500);
    DescretePathWithCurvature generatePathByLengthWithCurvature(double length, int initDistStep = 250, int traverseStep = 2000, bool end = true);

    Point2D getPoint(double t) const;
    Point2D getVelocity(double t) const;
    Point2D getAcceleration(double t) const;
 
    Point2D getP1();
    Point2D getP2();
    Point2D getC1();
    Point2D getC2();
};

