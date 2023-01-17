#include "include/math.hpp"
#include "include/structs.hpp"
#include <iostream>


//correct math
double Math::getCircumRadius(Point2D a, Point2D b, Point2D c){
    double a1 = b.distanceTo(c);
    double b1 = c.distanceTo(a);
    double c1 = a.distanceTo(b);
    
    double semiPerimeter = (a1 + b1 + c1) / 2.0;
    double area = sqrt(semiPerimeter * (semiPerimeter - a1) * (semiPerimeter - b1) * (semiPerimeter - c1));
    double radius = a1 * b1 * c1 / area / 4.0;
    return radius;
}


quadraticRoots Math::quadraticSolver(double a, double b, double c){
    double discrim = b * b - 4 * a * c;
    if (discrim < 0) return {};
    if (discrim > 0) return {(-b + sqrt(discrim)) / (2 * a), (-b - sqrt(discrim)) / (2 * a)};
    return {-b / (2 * a)};
}




double Math::findSmallestRoot(cubicRoots in){
    bool one = 0, two = 0, three = 0;

    if (in.root1.has_value() && in.root1 > 0) one = 1;
    if (in.root2.has_value() && in.root2 > 0) two = 1;
    if (in.root3.has_value() && in.root3 > 0) three = 1;

    if (one && two && three) return std::min(in.root1.value(), std::min(in.root2.value(), in.root3.value()));
    if (one && two) return std::min(in.root1.value(), in.root2.value());
    if (two && three) return std::min(in.root2.value(), in.root3.value());
    if (one && three) return std::min(in.root1.value(), in.root3.value());
    if (one) return in.root1.value();
    if (two) return in.root2.value();
    if (three) return in.root3.value();
    return -1000;
}

double Math::findSmallestRoot(quadraticRoots in){
    bool one = 0, two = 0;

    if (in.root1.has_value() && in.root1.value() > 0) one = 1;
    if (in.root2.has_value() && in.root2.value() > 0) two = 1;

    if (one && two) return std::min(in.root1.value(), in.root2.value());
    if (one) return in.root1.value();
    if (two) return in.root1.value();
    return -1000;
}


cubicRoots Math::cubicSolver(double a, double b, double c, double d){
    cubicRoots out;
    double disc, q, r, dum1, s, t, term1, r13;
    if (a == 0){
        std::cout << "this a quadratic bro";
        return {};
    }
    if (d == 0){
        std::cout << "one of the root is 0 just divide by x and make it a quadratic bruh";
        return {};
    }


    b /=a;
    c /=a;
    d /=a;

    q = (3.0*c - (b*b))/9.0;
    r = -(27.0*d) + b*(9.0*c - 2.0*(b*b));
    r /= 54.0;
    disc = q*q*q + r*r;
    term1 = (b/3.0);


    if (disc > 0) { // one root real, two are complex
        s = r + sqrt(disc);
        s = ((s < 0) ? -std::pow(-s, (1.0/3.0)) : std::pow(s, (1.0/3.0)));
        t = r - sqrt(disc);
        t = ((t < 0) ? -std::pow(-t, (1.0/3.0)) : std::pow(t, (1.0/3.0)));
        out.root1.emplace(-term1 + s + t);
        term1 += (s + t)/2.0;
        term1 = sqrt(3.0)*(-t + s)/2;
        return out;
    } 


    // The remaining options are all real
    if (disc == 0){ // All roots real, at least two are equal.
        r13 = ((r < 0) ? -std::pow(-r,(1.0/3.0)) : std::pow(r,(1.0/3.0)));
        out.root1 = -term1 + 2.0 * r13;
        out.root2 = -(r13 + term1);
        return out;
    } // End if (disc == 0)

    
    // Only option left is that all roots are real and unequal (to get here, q < 0)
    q = -q;
    dum1 = q*q*q;
    dum1 = acos(r/sqrt(dum1));
    r13 = 2.0*sqrt(q);
    out.root1 = -term1 + r13 * cos(dum1/3);
    out.root2 = -term1 + r13 * cos((dum1 + 2.0 * Math::pi)/3);
    out.root3 = -term1 + r13 * cos((dum1 + 4.0 * Math::pi)/3);
    return out;
}



double Math::getSmallestRootEquation(double a, double b, double c, double d){
    return findSmallestRoot(cubicSolver(a, b, c, d));
}
double Math::getSmallestRootEquation(double a, double b, double c){
    return findSmallestRoot(quadraticSolver(a, b, c));
}