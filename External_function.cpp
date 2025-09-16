#include <stdexcept>
#include <cmath>
#include "Geometry.h"


double round_(double d) {
    return std::round(d * 10) / 10;
}

Point intersection(const Line& l_1, const Line& l_2) {
    if (((l_1.b == l_2.b) && (l_2.b == 0)) || ((l_1.a == l_2.a) && (l_2.a == 0)))
        throw std::invalid_argument("line is not intersect");
    if (l_1.b == 0) {
        double x = -l_1.c / l_1.a;
        double y = (-l_2.c - l_2.a * x) / l_2.b;
        return { x,y };
    }
    else {
        double x = (l_2.b * l_1.c - l_2.c * l_1.b) / (l_1.b * l_2.a - l_2.b * l_1.a);
        double y = (-l_1.c - l_1.a * x) / l_1.b;
        return { x,y };
    }
}

double dist(const Point& p_1, const Point& p_2) {
    double result = std::sqrt(std::pow(p_2.x - p_1.x, 2) + std::pow(p_2.y - p_1.y, 2));
    return std::round(result * 10) / 10;
}

Point direct_vector(const Point& p_1, const Point& p_2) {
    double x = p_2.x - p_1.x;
    double y = p_2.y - p_1.y;
    return { x, y };
}

Line orthogonal(const Point& vector_dir, const Point& point) {
    // скалярное произведение ортогональных векторов == 0, учитывая это находим 
    // ортогональный вектор вектору vector_coordinate, он будет направляющим вектором
    // для серединного перпендикуляра стороне, на которой лежит center_edge
    double a = vector_dir.x;
    double b = vector_dir.y;
    double c = -a * point.x - b * point.y;
    return { a,b,c };
}

Line orthogonal_to_line(const Point& point_from, const Line& line_to) {
    // координаты вектора нормали прямой, являются одновременно координатами направляющего вектора
    // для прямой , проходящей из point_from to Line_to
    double a = line_to.b;
    double b = -line_to.a;
    double c = (-point_from.x * line_to.b) + (point_from.y * line_to.a);
    return Line(a, b, c);
}

double angles(const Point& p, const Point& p_2, const Point& p_3) {
    std::pair<Point, Point> pair = std::make_pair(direct_vector(p, p_2), direct_vector(p_2, p_3));
    std::pair<double, double> len = std::make_pair(dist(Point(0, 0), pair.first), dist(Point(0, 0), pair.second));

    double mult_coordinate = pair.first.x * pair.second.x + pair.first.y * pair.second.y;
    double mult_len = len.first * len.second;

    double angle_this = mult_coordinate / mult_len;

    return std::round(std::acos(angle_this) * 180 / PI);
}

bool verify_angles(const std::vector<Point>& v, const std::vector<Point>& v_) {
    for (size_t k = 2; k < v.size(); ++k) {
        if (angles(v[k - 2], v[k - 1], v[k]) != angles(v_[k - 2], v_[k - 1], v_[k]))
            return false;
    }

    return (angles(*(v.end() - 2), v.back(), v.front())
        != angles(*(v_.end() - 2), v_.back(), v_.front())) ? false : true;
}

bool verify_edges(const std::vector<Point>& v, const std::vector<Point>& v_) {
    for (size_t i = 1; i < v.size(); ++i) {
        if (round(dist(v[i - 1], v[i])) != round(dist(v_[i - 1], v_[i]))) {
            return false;
        }
    }
    return (round(dist(v.front(), v.back())) != round(dist(v_.front(), v_.back()))) ? false : true;
}

bool verify_relate_edges(const std::vector<Point>& v, const std::vector<Point>& v_) {
    double relate = dist(v[0], v[1]) / dist(v_[0], v_[1]);
    for (size_t i = 2; i < v.size(); ++i)
        if (dist(v[i - 1], v[i]) / dist(v_[i - 1], v_[i]) != relate)
            return false;

    return (dist(v.front(), v.back()) / dist(v_.front(), v_.back()) != relate) ? false : true;
}

int compare_sp(const void* p, const void* p2) {
    const Polygon::Size_plane* p_ = static_cast<const Polygon::Size_plane*>(p);
    const Polygon::Size_plane* p2_ = static_cast<const Polygon::Size_plane*>(p2);
    if (p_->area == p2_->area)
        return 0;
    return (p_->area < p2_->area) ? -1 : 1;
}