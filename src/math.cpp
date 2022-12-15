#include "math.hpp"
#include <cmath>


double Math::getCircumRadius(Point2D a, Point2D b, Point2D c)
{
    double a1 = b.distanceTo(c);
    double b1 = c.distanceTo(a);
    double c1 = a.distanceTo(b);

    double a2 = a1 * a1;
    double b2 = b1 * b1;
    double c2 = c1 * c1;

    Point2D pa = a * (a2 * (b2 + c2 - a2) / ((b1 + c1) * (b1 + c1) - a2) / (a2 - (b1 - c1) * (b1 - c1)));
    Point2D pb = b * (b2 * (a2 + c2 - b2) / ((a1 + c1) * (a1 * c1) - b2) / (b2 - (a1 - c1) * (a1 - c1)));
    Point2D pc = c * (c2 * (a2 + b2 - c2) / ((a1 + b1) * (a1 + b1) - c2) / (c2 - (a1 - b1) * (a1 - b1)));

    Point2D center = pa + pb + pc;
    return center.distanceTo(a);
}


double Math::getCircumRadius2(Point2D a, Point2D b, Point2D c){
    double a1 = b.distanceTo(c);
    double b1 = c.distanceTo(a);
    double c1 = a.distanceTo(b);
    
    double semiPerimeter = (a1 + b1 + c1) / 2.0;
    double area = sqrt(semiPerimeter * (semiPerimeter - a1) * (semiPerimeter - b1) * (semiPerimeter - c1));
    double radius = a1 * b1 * c1 / area / 4.0;
    return radius;
}