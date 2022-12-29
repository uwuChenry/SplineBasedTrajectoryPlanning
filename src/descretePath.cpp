#include "include/descretePath.hpp"
/*
void DescretePath::setDistance(double idistance){
    distance = idistance;
}
double DescretePath::getDistance(){
    return distance;
}
void DescretePath::setDeltaLength(double ilength){
    deltaLength = ilength;
}
double DescretePath::getDeltaLength(){
    return deltaLength;
}
void DescretePath::pushBack(Point2D point){
    path.push_back(point);
}
void DescretePath::popBack(){
    path.pop_back();
}
Point2D DescretePath::back(){
    return path.back();
}
int DescretePath::getSize(){
    return path.size();
}

double DescretePath::getCurvature(int index){
    if (index <= 0 || (long long unsigned int)index > path.size()){
        return 0;
    }
    double radius = Math::getCircumRadius2(path[index -1], path[index], path[index +1]);
    
    if (std::isnan(radius)){
        return 0;
    }
    return (1.0 / radius);
}
*/

void DescretePathWithCurvature::setDistance(double idistance){
    distance = idistance;
}

double DescretePathWithCurvature::getDistance(){
    return distance;
}

void DescretePathWithCurvature::setDeltaLength(double ilength){
    deltaLength = ilength;
}

double DescretePathWithCurvature::getDeltaLength(){
    return deltaLength;
}


int DescretePathWithCurvature::getSize(){
    return path.size();
}

double DescretePathWithCurvature::getCurvature(int index){
    if (index <= 0 || (long long unsigned int)index >= path.size()){
        return curvature[index];
    }
    double radius = Math::getCircumRadius(path[index -1], path[index], path[index +1]);
    bool isNeg = std::signbit(curvature[index]);
    if (isNeg) radius *= -1;
    if (std::isnan(radius)) return 0;
    return (1.0 / radius);
}