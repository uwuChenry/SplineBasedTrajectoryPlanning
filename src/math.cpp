#include "include/math.hpp"
#include "include/structs.hpp"
#include <iostream>


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


quadraticRoots Math::quadraticSolver(double a, double b, double c){
    double discrim = b * b - 4 * a * c;
    if (discrim < 0) return {};
    if (discrim > 0) return {(-b + sqrt(discrim)) / 2 * a, (-b - sqrt(discrim)) / 2 * a};
    return {-b / 2 * a};
}


cubicRoots Math::cubicSolver(double a, double b, double c, double d){
    cubicRoots out; 
    double A = a, B = b/3.0, C = c / 3.0, D = d;
    double H = A * C - B * B;
    double G = 2 * B * B * B - 3 * A * B * C + A * A * D;
    double discrim = G * G + 4 * H * H * H;
    std::cout << discrim << " discrim \n";
    double alpha = cbrt((G + sqrt(discrim)) / 2.0);
    //double thing = (G + sqrt(discrim)) / 2;
    //double thing2 = cbrt(thing);
    double y1 = alpha + H / alpha;
    double x1 = (y1 - B) / A;

    out.root1 = x1;
    std::cout << G << " test \n";
    //double a1 = 1;
    double b1 = y1;
    double c1 = 3 * H + y1 * b1;

    //quad discrim
    double num1 = b1 * b1 - 4 * c1;
    if (num1 < 0){
        double posNum = num1 * -1;
        double imagePart = sqrt(posNum) / 2;
        double num3 = (-1 * b1) / 2;
        num3 = (num3 - B)/A;
        imagePart = imagePart / A;
        out.root2 = -1;
        out.root3 = -1;
    } 
    else {
        double num11 = sqrt(num1) / 2;
        double real1 = (-1 * b1) / 2 - num11;
        double real2 = (-1 * b1) / 2 + num11;
        real1 = (real1 - B) / A;
        real2 = (real2 - B) / A;
        out.root2 = real1;
        out.root3 = real2;
    }
    return out;
}




double Math::findSmallestRoot(cubicRoots2 in){
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


cubicRoots2 Math::cubicSolver2(double a, double b, double c, double d){
    cubicRoots2 out;
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
    double disc, q, r, dum1, s, t, term1, r13;
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
        //dataForm.x1Re.value = -term1 + s + t;
        out.root1.emplace(-term1 + s + t);
        term1 += (s + t)/2.0;
        //dataForm.x3Re.value = dataForm.x2Re.value = -term1;
        term1 = sqrt(3.0)*(-t + s)/2;
        //dataForm.x2Im.value = term1;
        //dataForm.x3Im.value = -term1;
        return out;
    } 
    // End if (disc > 0)
    // The remaining options are all real
    //dataForm.x3Im.value = dataForm.x2Im.value = 0;
    if (disc == 0){ // All roots real, at least two are equal.
        r13 = ((r < 0) ? -std::pow(-r,(1.0/3.0)) : std::pow(r,(1.0/3.0)));
        //dataForm.x1Re.value = -term1 + 2.0*r13;
        out.root1 = -term1 + 2.0 * r13;
        out.root2 = -(r13 + term1);
        //dataForm.x3Re.value = dataForm.x2Re.value = -(r13 + term1);
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
    //dataForm.x1Re.value = -term1 + r13*Math.cos(dum1/3.0);
    //dataForm.x2Re.value = -term1 + r13*Math.cos((dum1 + 2.0*Math.PI)/3.0);
    //dataForm.x3Re.value = -term1 + r13*Math.cos((dum1 + 4.0*Math.PI)/3.0);
    return out;
}


cubicRoots Math::cubicSolver3(double a, double b, double c, double d){
    cubicRoots out;
    if (a == 0){
        std::cout << "this a quadratic bro";
        return {-1, -1, -1};
    }
    if (d == 0){
        std::cout << "one of the root is 0 just divide by x and make it a quadratic bruh";
    }
    b /=a;
    c /=a;
    d /=a;
    double disc, q, r, dum1, s, t, term1, r13;
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
        //dataForm.x1Re.value = -term1 + s + t;
        out.root1 = (-term1 + s + t);
        out.root2 = -1000;
        out.root3 = -1000;
        term1 += (s + t)/2.0;
        term1 = sqrt(3.0)*(-t + s)/2;
        return out;
    } 
    // End if (disc > 0)
    // The remaining options are all real
    if (disc == 0){ // All roots real, at least two are equal.
        r13 = ((r < 0) ? -std::pow(-r,(1.0/3.0)) : std::pow(r,(1.0/3.0)));
        out.root1 = -term1 + 2.0 * r13;
        out.root2 = -(r13 + term1);
        out.root3 = -1000;
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
    return findSmallestRoot(cubicSolver2(a, b, c, d));
}
double Math::getSmallestRootEquation(double a, double b, double c){
    return findSmallestRoot(quadraticSolver(a, b, c));
}