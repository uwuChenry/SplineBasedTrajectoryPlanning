#include "bezier.hpp"


double CubicBezier::getLength(int step){
    double out;
    double deltaStep = 1.0 / step;
    
    for (double i = 0; i < 1; i = (i + deltaStep))
    {
        out += Point2D::distanceBetween(getPoint(i), getPoint(i + deltaStep));
    }
    //std::cout << "get length length" << out;
    return out;
}

void CubicBezier::setPoints(Point2D start, Point2D control1, Point2D control2, Point2D end){
    this->p1 = start;
    this->c1 = control1;
    this->c2 = control2;
    this->p2 = end;
}
void CubicBezier::setPoints(Vector2D start, Vector2D end){
    end.addAngle(180);
    this->p1 = start.getPoint();
    this->p2 = end.getPoint();
    this->c1 = start.getSecondPoint();
    this->c2 = end.getSecondPoint();
    //setPoints(start.getPoint(), start.getSecondPoint(), end.getSecondPoint(), end.getPoint());
}

DescretePath CubicBezier::generatePathByStep(int step){
    DescretePath out;
    for (size_t i = 0; i < 1; i += 1 / step)
    {
        out.pushBack(getPoint(i));
    }
    return out;
}
DescretePath CubicBezier::generatePathByLength(int length, int initDistStep, int traverseStep, bool end){
    double totalDist = getLength(initDistStep);
    double distPerSegment = totalDist / std::ceil(totalDist / length);
    double traversed = 0;
    DescretePath out;
    out.setDeltaLength(length);
    out.setDistance(totalDist);
    out.pushBack(getPoint(0));
    for (size_t t = 0; t < 1000; t++)
    {
        traversed += getPoint(t / 1000).distanceTo(getPoint(t / 1000 + 1 / 1000));
        if (traversed >= distPerSegment)
        {
            traversed = 0;
            out.pushBack(getPoint(t / 1000));
        }
    }
    if (out.back().distanceTo(getPoint(1)) < distPerSegment / 2)
    {
        out.popBack();
    }
    if (end)
    {
        out.pushBack(getPoint(1));
    }
    std::cout << "finish gen by length \n";
    return out;
}
Point2D CubicBezier::getPoint(double t) const{
    return p1 * (1 - t) * (1 - t) * (1 - t) + c1 * 3 * (1 - t) * (1 - t) * t + c2 * 3 * (1 - t) * t * t + p2 * t * t * t;
}

Point2D CubicBezier::getVelocity(double t) const{
    return p1 * (-3 * t * t + 6 * t - 3) + c1 * (9 * t * t - 12 * t + 3) + c2 * (-9 * t * t + 6 * t) + p2 * (3 * t * t);
}

Point2D CubicBezier::getAcceleration(double t) const{
    return p1 * (-6 * t + 6) + c1 * (18 * t - 12) + c2 * (-18 * t + 6) + p2 * (6 * t);
}

Point2D CubicBezier::getP1(){
    return this->p1;
}
Point2D CubicBezier::getP2(){
    return this->p2;
}
Point2D CubicBezier::getC1(){
    return this->c1;
}
Point2D CubicBezier::getC2(){
    return this->c2;
}