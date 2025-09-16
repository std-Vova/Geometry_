#include <cmath>
#include <exception>
#include <stdexcept>
#include <iostream>
#include "Geometry.h"

Rectangle::Rectangle(const Point& point_0, const Point& point_2, double k) :
    Polygon{ point_0, Point(), point_2, Point() } {
    double angle = PI - 2 * std::atan(k);
    v[1] = v[0].rotate(center(), -angle);
    v[3] = v[2].rotate(center(), -angle);
}

Point Rectangle::center() const {
    return Point(v[2], v[0]);
}

std::pair<Line, Line> Rectangle::diagonals() const {
    auto pair = std::make_pair(Line(v[0], v[2]), Line(v[1], v[3]));
    return pair;
}

double Rectangle::perimeter() const {
    double result = dist(v[1], v[0]) + dist(v[1], v[2]);
    return result * 2;
}

double Rectangle::area() const {
    return dist(v[0], v[1]) * dist(v[1], v[2]);
}

bool Rectangle::is_congruent_to(const Shape& other) const {
    if (*this != other)
        return false;

    try {
        if (const Rectangle& shape = dynamic_cast<const Rectangle&>(other); &shape != nullptr) {
            return (dist(v[0], v[1]) == dist(shape.v[0], shape.v[1]) &&
                dist(v[1], v[2]) == dist(shape.v[1], shape.v[2])) ? true : false;

        }
        else {
            std::cout << "source type is null pointer\n";
            return false;
        }
    }
    catch (std::bad_cast& error) {
        std::cout << error.what() << '\n';
        return false;
    }
}

bool Rectangle::is_similar_to(const Shape& other) const {
    if (*this != other)
        return false;
    try {
        if (const Rectangle& shape = dynamic_cast<const Rectangle&>(other); &shape != nullptr) {
            double edge_this_1 = dist(v[0], v[1]);
            double edge_this_2 = dist(v[1], v[2]);

            double edge_other_1 = dist(shape.v[0], shape.v[1]);
            double edge_other_2 = dist(shape.v[1], shape.v[2]);

            return (std::round(edge_this_1 / edge_this_2) == std::round(edge_other_1 / edge_other_2)) ? true : false;
        }
        else {
            std::cout << "source type is null pointer\n";
            return false;
        }
    }
    catch (std::bad_cast& er) {
        std::cout << er.what() << '\n';
        return false;
    }
}

