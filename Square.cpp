#include <iostream>
#include "Geometry.h"

Square::Square(const Point& p_0, const Point& p_2) :
    Rectangle{ p_0, Point(), p_2, Point() } {
    double angle = PI - 2 * std::atan(1);
    v[1] = v[0].rotate(center(), -angle);
    v[3] = v[2].rotate(center(), -angle);
}

Circle Square::circumscribed_circle() const {
    Point center = this->center();
    double diagonal_len = dist(v[0], v[2]);
    return Circle(center, diagonal_len / 2);
}

Circle Square::inscribed_circle() const {
    Point center = this->center();
    double len_edge = dist(v[0], v[1]);
    return Circle(center, len_edge / 2);
}

double Square::perimeter() const {
    double result = dist(v[1], v[0]) * 4;
    return result;
}

bool Square::is_congruent_to(const Shape& other) const {
    if (*this != other)
        return false;

    try {
        if (const Square& shape = dynamic_cast<const Square&>(other); &shape != nullptr) {
            return (dist(v[0], v[1]) == dist(shape.v[0], shape.v[1])) ? true : false;
        }
        else {
            std::cout << "source type is null pointer";
            return false;
        }
    }
    catch (std::bad_cast& error) {
        std::cout << error.what();
        return false;
    }
}

bool Square::is_similar_to(const Shape& other) const {
    return (*this == other) ? true : false;
}