#pragma once


class Point2D
{
    public:
    double x = 0;
    double y = 0;


    Point2D() = default;
    Point2D(double ix, double iy);
    

    Point2D operator+(const Point2D &rhs);
    Point2D operator-(const Point2D &rhs);
    Point2D operator*(double scalar) const;
    Point2D operator/(double scalar) const;

    double getX();
    double getY();
    double getMagnitude();
    void printXandY();


    
    double getAngle();
    double distanceTo(Point2D b);
    double getAngleTo(Point2D b);
    double dot(Point2D b);

    static Point2D subtract(Point2D a, Point2D b);
    static double distanceBetween(Point2D a, Point2D b);
    static double dot(Point2D a, Point2D b);
    static double angleBetween(Point2D a, Point2D b);
};

