#include <cmath>
#include "Geometry.h"

Point::Point() : x(0.0), y(0.0) {}

Point::Point(double x, double y) : x(x), y(y) {}

Point::Point(const Point& other) : x(other.x), y(other.y) {}

Point::Point(const Point& p_1, const Point& p_2) :
    x((p_1.x + p_2.x) / 2), y((p_1.y + p_2.y) / 2) {}

bool Point::operator==(const Point& other) const {
    return (x == other.x && y == other.y) ? true : false;
}

bool Point::operator<(const Point& other) const {
    if (x < other.x)
        return true;
    if (x > other.x)
        return false;
    if (y < other.y)
        return true;
    if (y > other.y)
        return false;
    return false;
}

bool Point::operator>(const Point& other) const {
    return (*this == other) ? false : !(*this < other);
}

std::partial_ordering Point::operator<=>(const Point& other) const {
    if (*this < other)
        return std::partial_ordering::less;
    if (*this == other)
        return std::partial_ordering::equivalent;
    if (*this > other)
        return std::partial_ordering::greater;
    return std::partial_ordering::unordered;
}

Point& Point::operator=(const Point& other) {
    x = other.x;
    y = other.y;
    return *this;
}

Point& Point::operator-=(const Point& other) {
    x -= other.x;
    y -= other.y;
    return *this;
}

Point operator-(const Point& p, const Point& p2) {
    Point result = p;
    result -= p2;
    return result;
}

Point Point::rotate(const Point& point, double angle) const {
    double x_ = (x - point.x) * std::cos(angle) - (y - point.y) * std::sin(angle) + point.x;
    double y_ = (x - point.x) * std::sin(angle) + (y - point.y) * std::cos(angle) + point.y;
    return Point(x_, y_);
}

Point& Point::operator*=(double coef) {
    x *= coef;
    y *= coef;
    return *this;
}

Point operator*(const Point& point, double coef) {
    Point temp = point;
    temp *= coef;
    return temp;
}

Point& Point::operator+=(const Point& point) {
    x += point.x;
    y += point.y;
    return *this;
}

Point operator+(const Point& p, const Point& p2) {
    Point temp = p;
    temp += p2;
    return temp;
}

void Point::reflect_by_point(const Point& point) {
    *this = (point * 2 - *this);
}

void Point::reflect_by_line(const Line& line) {
    Line ort_line_from_point = orthogonal_to_line(*this, line);
    Point crossing_point = intersection(line, ort_line_from_point);
    double distance = dist(*this, crossing_point);
    Point coeff = crossing_point - *this;
    *this = crossing_point + coeff;
}

void Point::homotetia(const Point& center, double coeff) {
    Point len_coeff = (*this - center) * coeff;
    *this = *this + len_coeff;
}