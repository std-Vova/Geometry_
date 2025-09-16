#include <iostream>
#include "Geometry.h"
 
Circle::Circle(const Point& center, double radius) :
    Elipsoid(Point(center.x + radius, center.y), Point(center.x - radius, center.y), radius),
    center(center), radius(radius) {}

bool Circle::operator==(const Circle& other) const {
    return (center == other.center && radius >= other.radius - 0.1 &&
        radius <= other.radius + 0.1) ? true : false;
}

double Circle::perimeter() const {
    return 2 * PI * radius;
}

double Circle::area() const {
    return PI * (radius * radius);
}

bool Circle::is_congruent_to(const Shape& other) const {
    if (*this != other)
        return false;

    try {
        if (const Circle& shape = dynamic_cast<const Circle&>(other); &shape != nullptr)
            return (radius == shape.radius) ? true : false;
        else {
            std::cout << "source type is null poiner";
            return false;
        }
    }
    catch (std::bad_cast& error) {
        std::cout << error.what();
        return false;
    }
}

bool Circle::is_similar_to(const Shape& other) const {
    return (*this != other) ? false : true;
}

bool Circle::contains_point(const Point& point) const {
    return (::dist(center, point) <= radius) ? true : false;
}

void Circle::rotate(const Point& point, double angle) {
    center.rotate(point, angle);
}

void Circle::reflect_by_point(const Point& point) {
    center.reflect_by_point(point);
}

void Circle::reflect_by_line(const Line& line) {
    center.reflect_by_line(line);
}

void Circle::scale(const Point& center, double coeff) {
    this->center.homotetia(center, coeff);
    radius *= coeff;
}
