#include "include/bezier.hpp"


double CubicBezier::getLength(int step){
    double out;
    double deltaStep = 1.0 / step;
    
    for (double i = 0; i < 1; i = (i + deltaStep)){
        out += Point2D::distanceBetween(getPoint(i), getPoint(i + deltaStep));
    }
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
}

DescretePathWithCurvature CubicBezier::generatePathByStep(int step){
    DescretePathWithCurvature out;
    for (size_t i = 0; i < 1; i += 1 / step)
    {
        out.path.push_back(getPoint(i));
        out.curvature.push_back(getCurvature(i));
    }
    return out;
}


DescretePathWithCurvature CubicBezier::generatePathByLengthWithCurvature(double length, int initDistStep, int traverseStep, bool end){
    double totalDist = getLength(initDistStep);
    double distPerSegment = totalDist / std::ceil(totalDist / length);
    double traversed = 0;
    DescretePathWithCurvature out;
    out.setDistance(totalDist);
    out.path.push_back(getPoint(0));
    out.curvature.push_back(getCurvature(0));

    double seg = 0;

    for (double t = 0; t < traverseStep; t++){
        traversed += getPoint(t / traverseStep).distanceTo(getPoint(t / traverseStep + 1.0/traverseStep));
        if (traversed >= distPerSegment){
            seg += traversed;
            traversed = 0;
            out.path.push_back(getPoint(t / traverseStep));
            out.curvature.push_back(getCurvature(t / traverseStep));
        }
    }
    if (out.path.back().distanceTo(getPoint(1)) < distPerSegment / 2){
        out.path.pop_back();
        out.curvature.pop_back();
    }
    if (end){
        out.path.push_back(getPoint(1));
        out.curvature.push_back(getCurvature(1));
        
    }   
    out.setDeltaLength(seg/out.getSize());
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

double CubicBezier::getCurvature(double t) const{
    Point2D v = getVelocity(t);
    Point2D a = getAcceleration(t);
    double vmag = v.getMagnitude();
    double out =  (v.getX() * a.getY() - v.getY() * a.getX()) / (vmag * vmag * vmag);
    
    if (std::isnan(out)) return 0;
    else return out;
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