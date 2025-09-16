#include <iostream>
#include <cmath>
#include "Geometry.h"

Circle Triangle::circumscribed_circle() const {
    // точка пересечения серединных перпендикуляров - центр описанной окружности 
    Point center_v0_v1 = Point(v[0], v[1]);
    Point center_v1_v2 = Point(v[1], v[2]);

    Point vector_v0_v1 = Point(v[1] - v[0]);
    Point vector_v1_v2 = Point(v[2] - v[1]);

    Line orthogonal_to_vector_v0_v1 = orthogonal(vector_v0_v1, center_v0_v1);
    Line orthogonal_to_vector_v1_v2 = orthogonal(vector_v1_v2, center_v1_v2);

    Point center_circle = intersection(orthogonal_to_vector_v0_v1, orthogonal_to_vector_v1_v2);
    double radius = dist(center_circle, v[0]);
    return Circle(center_circle, radius);
}

Circle Triangle::inscribed_circle() const {
    // S = rp
    // p - полупериметр

    double a = dist(v[0], v[1]);
    double b = dist(v[1], v[2]);
    double c = dist(v[2], v[0]);
    double p = a + b + c;
    double p_ = p / 2;

    double x = ((a * v[2].x) + (b * v[0].x) + (c * v[1].x)) / p;
    double y = ((a * v[2].y) + (b * v[0].y) + (c * v[1].y)) / p;
    double s = std::sqrt(p_ * (p_ - a) * (p_ - b) * (p_ - c));
    double radius = s / p_;
    return { Point(x,y), radius };
}

Point Triangle::centroid() const {
    Point center_1 = Point(v[0], v[1]);
    Point center_2 = Point(v[1], v[2]);

    Line first(v[0], center_2);
    Line second(v[2], center_1);

    Point result = intersection(first, second);

    return result;
}

Point Triangle::ortho_center() const {
    Point circum_center = circumscribed_circle().center;
    Point centroid_ = centroid();
    double x = (3 * centroid_.x - 2 * circum_center.x);
    double y = (3 * centroid_.y - 2 * circum_center.y);
    return { x, y };
}

Line Triangle::euler_line() const {
    Point circum_center = inscribed_circle().center;
    Point ortho_cent = ortho_center();

    return { circum_center, ortho_cent };
}

Circle Triangle::nine_points_circle() const {
    Point circum_center = circumscribed_circle().center;
    Point ort_center = ortho_center();
    Point center_euler_circle = Point(circum_center, ort_center);
    double radius_euler_circle = circumscribed_circle().radius / 2;
    return { center_euler_circle, radius_euler_circle };
}

double Triangle::perimeter() const {
    double result = dist(v[0], v[1]) + dist(v[1], v[2]) + dist(v[2], v[0]);
    return result;
}

double Triangle::area() const {
    Line line = Line(v[0], v[1]);
    double k = (-line.c - (line.a * v[0].x)) / line.b;
    double ort_k = -1 / k;
    double b = v[2].y - k * v[2].x;

    double c = b;
    double a = ort_k;
    double b_ = -1;
    Line ort_line(ort_k, b_, c);
    Point intersect = intersection(line, ort_line);
    double dist_1 = dist(v[0], v[1]);
    double dist_2 = dist(v[2], intersect);
    return (dist_1 * dist_2) * 0.5;
}

bool Triangle::is_congruent_to(const Shape& other) const {
    if (*this != other)
        return false;
    try {
        if (const Triangle& shape = dynamic_cast<const Triangle&>(other); &shape != nullptr) {
            return (verify_edges(v, shape.v)) ? true : false;
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

bool Triangle::is_similar_to(const Shape& other) const {
    if (*this != other)
        return false;

    try {
        if (const Triangle& shape = dynamic_cast<const Triangle&>(other); &shape != nullptr) {
            return (verify_relate_edges(v, shape.v)) ? true : false;
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

bool Triangle::contains_point(const Point& point) const {
    return triangle_test_convex(point);
    return true;
}





