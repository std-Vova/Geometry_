#include <iostream>
#include <stdexcept>
#include "Geometry.h"


Elipsoid::Elipsoid(const Point& p1, const Point& p2, double dist) : focus_left(p1), focus_right(p2), dist(dist) {}

std::pair<const Point&, const Point&> Elipsoid::focuses() const {
    std::pair<const Point&, const Point&> pair = std::make_pair(focus_left, focus_right);
    return pair;
}

std::pair<Line, Line> Elipsoid::directrices() const {
    // x = a / eccentricity
    // -x = a / eccentricity
    double dist_from_centr_to_a = dist / 2;  // a
    double x_directricex = dist_from_centr_to_a / eccentricity();
    Point right_directris_1(x_directricex, focus_right.y);
    Point right_directris_2(x_directricex, focus_right.y + 5);

    Point left_directris_1(-x_directricex, focus_left.y);
    Point left_directris_2(-x_directricex, focus_left.y + 5);

    std::pair<Line, Line> pair = std::make_pair(Line(left_directris_1, left_directris_2), Line(right_directris_1, right_directris_2));
    return pair;  // RVO will perform 
}

double Elipsoid::eccentricity() const {
    // точка лежит на эллипсе, если сумма расстояний = 2а => что а = сумма расстояний до фокусов / 2
    // epsilon = c / a
    double dist_beetwen_focus = focus_right.x - focus_left.x;
    double dist_from_centr_to_focus = dist_beetwen_focus / 2;    // c 

    double dist_from_centr_to_a = dist / 2;  // a
    double eccentricity = dist_from_centr_to_focus / dist_from_centr_to_a;
    return eccentricity;
}

Point Elipsoid::center() const {
    double dist_beetwen_focus = focus_right.x - focus_left.x;
    double dist_from_centr_to_focus = dist_beetwen_focus / 2;   // c 
    double x_centr = focus_right.x - dist_from_centr_to_focus;
    return (Point(x_centr, focus_right.y));
}

double Elipsoid::perimeter() const {
    // p = Pi * a * b
    // c = sqrt(a^2 - b^2)
    //double c = std::sqrt(std::pow(a, 2) - std::pow(b, 2));
    double a = dist / 2;
    double c = ::dist(focus_left, focus_right) / 2;
    double b = std::sqrt(std::pow(a, 2) - std::pow(c, 2));

    return PI * (a + b);
}

double Elipsoid::area() const {
    double a = dist / 2;
    double c = ::dist(focus_left, focus_right) / 2;
    double b = std::sqrt(std::pow(a, 2) - std::pow(c, 2));
    return PI * a * b;
}

bool Elipsoid::is_congruent_to(const Shape& other) const {
    if (*this != other)
        return false;

    try {
        if (const Elipsoid& shape = dynamic_cast<const Elipsoid&>(other); &shape != nullptr)
            return (::dist(focus_left, focus_right) == ::dist(shape.focus_left, shape.focus_right)) ? true : false;
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

bool Elipsoid::is_similar_to(const Shape& other) const {
    if (*this != other)
        return false;

    const Elipsoid& shape = dynamic_cast<const Elipsoid&>(other);
    return (eccentricity() == shape.eccentricity()) ? true : false;
}

bool Elipsoid::contains_point(const Point& point) const {
    return (::dist(point, focus_left) + ::dist(point, focus_right) <= dist) ? true : false;
}

void Elipsoid::rotate(const Point& point, double angle) {
    focus_left.rotate(point, angle);
    focus_right.rotate(point, angle);
}

void Elipsoid::reflect_by_point(const Point& point) {
    focus_left.reflect_by_point(point);
    focus_right.reflect_by_point(point);
}

void Elipsoid::reflect_by_line(const Line& line) {
    focus_left.reflect_by_line(line);
    focus_right.reflect_by_line(line);
}

void Elipsoid::scale(const Point& center, double coeff) {
    focus_left.homotetia(center, coeff);
    focus_right.homotetia(center, coeff);
    dist *= coeff;
}