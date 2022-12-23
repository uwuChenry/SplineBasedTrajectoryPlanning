#include "include/math.hpp"


//correct math
double Math::getCircumRadius2(Point2D a, Point2D b, Point2D c){
    double a1 = b.distanceTo(c);
    double b1 = c.distanceTo(a);
    double c1 = a.distanceTo(b);
    
    double semiPerimeter = (a1 + b1 + c1) / 2.0;
    double area = sqrt(semiPerimeter * (semiPerimeter - a1) * (semiPerimeter - b1) * (semiPerimeter - c1));
    double radius = a1 * b1 * c1 / area / 4.0;
    return radius;
}



/*
bool Math::isLeft(Point2D first, Point2D second, Point2D third){
    Point2D thing = second - first;
    double x = third.getX();
    double y = third.getY();
    if (x * thing.getY() - y * thing.getX() > 0) return false;
    else return true;
}*/