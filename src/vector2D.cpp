#include "vector2D.hpp"




Point2D Vector2D::getSecondPoint(){
    return Point2D(x + mag * cos(angle * Math::pi / 180), y + mag * sin (angle * Math::pi / 180));
}

Point2D Vector2D::getPoint(){
    return Point2D(x, y);
}

void Vector2D::addAngle(double relativeAngle){
    angle += relativeAngle;
}
