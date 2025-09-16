#include "Geometry.h"

//Ax + By + C = 0
Line::Line() : a(1), b(1), c(0) {}

Line::Line(Point M1, Point M2) : Line() {
    // x - x1 / ax = y - y1 / ay
    Point vector_ = M2 - M1;
    a = vector_.y;
    b = -vector_.x;
    c = -M1.x * vector_.y + M1.y * vector_.x;
}

Line::Line(double k, double b) : Line(Point(b, 0), k) {}

Line::Line(Point M1, double k) : Line() {
    // общее уравнение получается: xk - y + b = 0, тогда 
    // b = y - kx
    double b = M1.y - k * M1.x;
    a = k;
    b = -1;
    c = b;
}

Line::Line(double a, double b, double c) : a(a), b(b), c(c) {}

bool Line::operator==(const Line& other) const {
    return ((a / -b == other.a / -other.b) && (c / -b == other.c / -other.b));
}

bool Line::operator!=(const Line& other) const {
    return !(*this == other);
}