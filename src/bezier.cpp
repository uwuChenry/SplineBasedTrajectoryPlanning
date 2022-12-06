#include "bezier.hpp"


double CubicBezier::getLength(int step){
    double out;
    double deltaStep = 1.0 / step;
    
    std::cout<<"ahahaha";
    std::cout<< deltaStep;
    for (double i = 0; i < 1; i = (i + deltaStep))
    {
        out += Point2D::distanceBetween(getPoint(i), getPoint(i + deltaStep));
    }
    std::cout<<"ahahaha";
    
    return out;
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
    std::cout << "finish gen by length";
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
