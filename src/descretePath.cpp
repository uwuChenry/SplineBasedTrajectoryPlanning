#include "descretePath.hpp"

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

