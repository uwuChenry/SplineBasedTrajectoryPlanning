#pragma once

#include <cmath>
#include "point2D.hpp"
#include "structs.hpp"
#include <optional>


struct cubicRoots{
    std::optional<double> root1;
    std::optional<double> root2;
    std::optional<double> root3;
};

struct quadraticRoots{
    std::optional<double> root1;
    std::optional<double> root2;
};

class Math{
    public:
    static double getCircumRadius(Point2D a, Point2D b, Point2D c);

    static constexpr double pi = 3.1415926535897932384626433;
    
    static cubicRoots cubicSolver(double a, double b, double c, double d);

    static quadraticRoots quadraticSolver(double a, double b, double c);
    static double findSmallestRoot(cubicRoots in);
    static double findSmallestRoot(quadraticRoots in);
    static double getSmallestRootEquation(double a, double b, double c, double d);
    static double getSmallestRootEquation(double a, double b, double c);
};