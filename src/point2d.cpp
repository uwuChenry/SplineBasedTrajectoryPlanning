#include "include/point2D.hpp"
#include <cmath>
#include <iostream>
#include <string.h>


Point2D::Point2D(double ix, double iy){
    this->x = ix;
    this->y = iy;
}

double Point2D::getX(){
    return this->x;
}

double Point2D::getY(){
    return this->y;
}


double Point2D::getAngle(){
    return atan2(this->y, this->x);
}

double Point2D::getAngleTo(Point2D b){
    return angleBetween(*this, b);
}


double Point2D::distanceTo(Point2D b){
    return distanceBetween(*this, b);
}


Point2D Point2D::subtract(Point2D a, Point2D b){
    return Point2D((a.x - b.x), (a.y - b.y));
}


double Point2D::distanceBetween(Point2D a, Point2D b){
    auto dx = a.x - b.x;
    auto dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double Point2D::getMagnitude(){
    return hypot(x, y);
}

double Point2D::angleBetween(Point2D a, Point2D b){
    auto c = subtract(a, b);
    return atan2(c.y, c.x);
}

double Point2D::dot(Point2D a, Point2D b){
    return (a.x * b.x + a.y * b.y);
}

double Point2D::dot(Point2D b){
    return dot(*this, b);
}


Point2D Point2D::operator+(const Point2D &rhs){
    return Point2D(x + rhs.x, y + rhs.y);
}


Point2D Point2D::operator-(const Point2D &rhs){
    return Point2D(x - rhs.x, y - rhs.y);
}


Point2D Point2D::operator*(double scalar) const {
    return {x * scalar, y * scalar};
}


Point2D Point2D::operator/(double scalar) const {
    return {x / scalar, y / scalar};
}

void Point2D::printXandY(){
    std::cout << "x: " << this->x << " y: " << this->y << std::endl;
}