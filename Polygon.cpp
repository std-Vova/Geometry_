#include <iostream>
#include <vector>
#include <array>
#include <map>
#include <initializer_list>
#include <cmath>
#include <iomanip>
#include <exception>
#include <algorithm>
#include <cstdlib>
#include "Geometry.h"


//                          Alliace for Polygon

using type_polygon = Polygon::PolygonClass;

//                          POLYGON
Polygon::Polygon(size_t size) : v(size) {}

Polygon::Polygon(const std::vector<Point>& vec) : Polygon(1) {
    v[0] = vec[0];
    for (size_t i = 1; i < vec.size(); ++i) {
        if (vec[i] == vec[i - 1])
            continue;
        v.push_back(vec[i]);
    }
}

Polygon::Polygon(const std::initializer_list<Point>& list) : Polygon(1) {
    v[0] = *list.begin();
    for (auto iter = list.begin() + 1; iter != list.end(); ++iter) {
        if (*iter == *(iter - 1))
            continue;
        v.push_back(*iter);
    }
}

int Polygon::vertices_count() const {
    return v.size();
}

const std::vector<Point>& Polygon::get_vertices() const {
    return v;
}

bool Polygon::is_convex() const {
    switch (type_polygon()) {
    case type_polygon::ConvexCCW:
        return true;
        break;
    case type_polygon::ConvexCW:
        return true;
        break;
    case type_polygon::DegenerateConvex:
        return true;
        break;
    case type_polygon::NotConvex:
        return false;
        break;
    case type_polygon::NotConvexDegenerate:
        return false;
        break;
    }
}

// Поддерживаем 4 условия
// 1) Дана последовательность вершин: P = p1,p2,pn... n - целое
// 2) Последовательные вершины различны т.е pi != pi + 1, a (pn + 1 == p1)
// 3) Лексикографически меньшая вершина: p < q если (p.x < q.x) || ((p.x == q.x) && (p.y < q.y))
// 4) Все выпуклые многоугольники монотонны т.е, координата х сначала монотонно убывает(возрастает), 
// а, затем монотонно возрастает(убывает) т.е
// найдейтся такая pj при которой (1 <= i < j) => pi < pi + 1 и
// при (j <= i <= n) => pi + i < pi;

// Тогда для вершин с координатами pi(xi, yi) :
// d(i) = (x(i - 1) - xi) * (yi - y(i + 1)) - (y(i - 1) - yi) * (xi - x(i + 1));
// P обозначается левовыпуклой(против часой стрелки) если:
// для любого i: 1 <= i <= n : d(i) <= 0 && найдется такой i: 1 <= i <= n, что d(i) < 0;
// P обозначатеся правовыпуклой(по часовой стрелке) если: 
// для любого i: 1 <= i <= n : d(i) >= 0 && найдется такой i: 1 <= i <= n, что d(i) > 0;
// P обозначается вырожденно - выпуклой если:
// для любого i: 1 <= i <= n : d(i) == 0;
// P обозначается не выпуклым если: 
// существует такой i: 1 <= i <= n : d(i) < 0  существует такой i: 1 <= i <= n && d(i) > 0;

// Если последовательность удовлетворяет 4 условию, но точки лежат на одной прямой, то многоугольник - выпукло вырожденный
// Если 2 и 3 условия не выполняются из - за одинаковости точек, то одинаковые точки пропускаются.
// Если 4 условие не выполняется, то многоугольник - не выпуклый
// Если точки лежат на одной прямой, но не удовлетворяют 4 условию, то многоугольник 
// обозначается не выпуклый - вырожденный.

Polygon::PolygonClass Polygon::type_polygon() const {
    if (v.size() < 3)
        return Polygon::PolygonClass::DegenerateConvex;

    struct Box {
        mutable Point first;
        mutable Point second;
        mutable Point save_second;
        mutable Point third;
        mutable size_t changed_monotonic = 0;
        mutable int this_turn = 0;
        mutable int current_turn = 0;
        mutable int this_monotonic = 0;
        mutable int current_monotonic = 0;
    } box;
    box.first = v[0];

    size_t count = 1;
    while (v[count] == box.first)
        ++count;
    if (count == v.size() || count == v.size() - 1)
        return Polygon::PolygonClass::DegenerateConvex;

    box.second = v[count];
    box.save_second = box.second;

    box.current_monotonic = compare(box.first, box.second);  // вычисление изначальной монотонности

    for (; count < v.size(); ++count) {
        if (v[count] == box.second)   // пропускаем одинаковые следующие точки
            continue;

        box.third = v[count];
        if (check_conditional_and_shift(box))
            return type_polygon::NotConvex;
    }
    if (v.back() != v.front()) {
        box.third = v[0];
        if (check_conditional_and_shift(box))
            return type_polygon::NotConvex;
    }
    box.third = box.save_second;
    if (check_conditional_and_shift(box))
        return type_polygon::NotConvex;


    if (box.changed_monotonic > 2) {
        return (box.current_turn) ? type_polygon::NotConvex : type_polygon::NotConvexDegenerate;
    }
    if (box.current_turn > 0)
        return type_polygon::ConvexCCW;
    if (box.current_turn < 0)
        return type_polygon::ConvexCW;
    return type_polygon::DegenerateConvex;
}

int Polygon::check_side(Point p, Point q, Point r) const {
    double result = 0;
    result = (p.x - q.x) * (q.y - r.y) - (p.y - q.y) * (q.x - r.x);
    if (result > 0)  // qr повернула против часовой стрелки
        return 1;
    if (result < 0) // qr повернула по часовой стрелке
        return -1;
    return 0;
}

int Polygon::compare(Point p, Point q) const {
    if (p.x > q.x)
        return 1;  // p > 1
    if (p.x < q.x)
        return -1;  // p < 1
    if (p.y > q.y)
        return 1;    // p > 1
    if (p.y < q.y)
        return -1;   // p < 1
    return 0; // p == q
}

template <typename T>
bool Polygon::check_conditional_and_shift(const T& box) const {
    if ((box.this_monotonic = compare(box.second, box.third)) == -box.current_monotonic) {
        ++box.changed_monotonic;
    }
    box.current_monotonic = box.this_monotonic;

    if (box.this_turn = check_side(box.first, box.second, box.third)) {
        if (box.this_turn == -box.current_turn)
            return true;
    }
    box.current_turn = box.this_turn;

    box.first = box.second;
    box.second = box.third;
    return false;
}


bool Polygon::is_convex_optimized() const {
    switch (type_polygon_optimized()) {
    case type_polygon::ConvexCCW:
        return true;
        break;
    case type_polygon::ConvexCW:
        return true;
        break;
    case type_polygon::DegenerateConvex:
        return true;
        break;
    case type_polygon::NotConvex:
        return false;
        break;
    case type_polygon::NotConvexDegenerate:
        return false;
        break;
    }
}

// Оптимизация для is_convex
// 1) Вместо хранения трех переменных: first, second, third - будем зранить delta:
// delta_previouse(second - first), delta_current(third - second);
// 2) Это позволить в операции compare сравнивать всего 2 числа с нулем, а не 4 числа друг с другом
// 3) Оперфция check_conditional будет выпонять 3 подоперации вместо 4
// 4) Для операции нахождения поворота будет применяться векторное произведение дельт, что быстрее, чем
// перемножать и вычитать точки на каждой итерации.
// 5) Если необходимо в ответе делать различие между NotConvex, NotConvexDegenerate 
// эта оптимизация может быть не полезна
// 6) Каждый раз переменная monotonic_change увеличивается, это замедляет алгоритм для 
// выпуклым многоугольников, но позволяет быстрее выйти из него при monotonic_change > 2 т.е 
// не выполняется условие 4

Polygon::PolygonClass Polygon::type_polygon_optimized() const {
    if (v.size() < 3)
        return type_polygon::DegenerateConvex;

    int initial_turn = 0;
    int current_turn;
    int initial_monotonic = 0;
    int current_monotonic;
    int changed_monotonic = 0;

    Point delta_prev;
    Point delta_curr;
    //Point save_second;
    // закоменнированные строки, это защита от повторяющихся вершин в данном наборе, если бы это была внешняя функция
    // объект класса гарантирует, что не хранит подряд одинаковых точек
    /*
    size_t i = 1;
    for (; i < v.size(); ++i) {
        delta_prev = v[i] - v[0];

        if (delta_prev.x || delta_prev.y)
            break;
    }
    if (i == v.size() || i == v.size() - 1)
        return type_polygon::DegenerateConvex;

    save_second = v[i];   // следующая точка после первой, отличная от неё
    */
    delta_prev = v[1] - v[0];
    initial_monotonic = compare_optimazed(delta_prev);

    //for (; i + 1 < v.size() + 1; ++i)

    for (size_t i = 2; i < v.size(); ++i) {
        delta_curr = v[i] - v[i - 1];

        //if (delta_curr == Point(0.0,0.0))
            //continue;

        if (check_conditional_and_shift_optimize
        (delta_prev, delta_curr,
            initial_turn, current_turn,
            initial_monotonic, current_monotonic,
            changed_monotonic)) {

            return type_polygon::NotConvex;
        }
        delta_prev = delta_curr;
    }

    if (v.back() != v.front()) {
        delta_curr = v.front() - v.back();
        if (check_conditional_and_shift_optimize
        (delta_prev, delta_curr,
            initial_turn, current_turn,
            initial_monotonic, current_monotonic,
            changed_monotonic))
            return type_polygon::NotConvex;

        //delta_curr = save_second - v[0];
        delta_curr = v[1] - v[0];
        if (check_conditional_and_shift_optimize
        (delta_prev, delta_curr,
            initial_turn, current_turn,
            initial_monotonic, current_monotonic,
            changed_monotonic))

            return type_polygon::NotConvex;
    }
    else {
        //delta_curr = save_second - v[0];
        delta_curr = v[1] - v[0];
        if (check_conditional_and_shift_optimize
        (delta_prev, delta_curr,
            initial_turn, current_turn,
            initial_monotonic, current_monotonic,
            changed_monotonic))

            return type_polygon::NotConvex;
    }

    if (changed_monotonic > 2)
        return (initial_turn) ? type_polygon::NotConvex : type_polygon::NotConvexDegenerate;
    if (initial_turn > 0)
        return type_polygon::ConvexCCW;
    if (initial_turn < 0)
        return type_polygon::ConvexCW;

    return type_polygon::DegenerateConvex;
}

int Polygon::compare_optimazed(Point delta) const {
    return (delta.x > 0) ? 1 : // second point > first point
        (delta.x < 0) ? -1 : // second point < first point
        (delta.y > 0) ? 1 : // first.x == second.x, second point > first point
        (delta.y < 0) ? -1 : // first.x == second.x, second point < first point 
        0;  // first.x == second.x && first.y == second.y
}

int Polygon::check_turn_optimized(Point delta_prev, Point delta_curr) const {
    return (delta_prev.x * delta_curr.y) - (delta_prev.y * delta_curr.x);
}

bool Polygon::check_conditional_and_shift_optimize
(Point& delta_prev, Point& delta_curr, int& initial_turn, int& current_turn, int& initial_monotonic, int& current_monotonic, int& changed_monotonic) const {
    if ((current_monotonic = compare_optimazed(delta_curr)) == -initial_monotonic) {
        ++changed_monotonic;
        // Следующие условие будет оптимизировать, если мы не хотим иметь разницы 
            // между NotConvex and NotConvexDegenerate

            //if (changed_monotonic > 2)
                //return type_polygon::NotConvex;

    }
    initial_monotonic = current_monotonic;
    current_turn = check_turn_optimized(delta_prev, delta_curr);

    if (current_turn) {
        if (current_turn > 0) {
            if (initial_turn < 0)
                return true;
            else {
                initial_turn = current_turn;
                return false;
            }
        }
        if (current_turn < 0) {
            if (initial_turn > 0)
                return true;
            else {
                initial_turn = current_turn;
                return false;
            }
        }
    }
    return false;
}

double Polygon::perimeter() const {
    std::vector<double> vec;
    double result = 0;
    for (int i = 1; i < vertices_count(); ++i) {
        result += (dist(v[i], v[i - 1]));
    }
    result += dist(v.back(), v.front());
    return result;
}

double Polygon::area() const {
    double a = 0;
    double b = 0;
    for (size_t i = 1; i < vertices_count(); ++i) {
        a += (v[i].y * v[i - 1].x);
        b += (v[i].x * v[i - 1].y);
    }
    a += v[0].y * v.back().x;
    b += v[0].x * v.back().y;
    return std::abs(a - b) / 2;
}

bool Polygon::is_congruent_to(const Shape& other) const {
    if (*this != other)
        return false;
    try {
        if (const Polygon& poly = dynamic_cast<const Polygon&>(other); &poly != nullptr) {
            if (v.size() != poly.vertices_count())
                return false;

            if (verify_angles(v, poly.v) == false)
                return false;

            return (verify_edges(v, poly.v)) ? true : false;

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

bool Polygon::is_similar_to(const Shape& other) const {
    if (*this != other)
        return false;
    try {
        if (const Polygon& poly = dynamic_cast<const Polygon&>(other); &poly != nullptr) {
            if (v.size() < poly.vertices_count())
                return false;

            // угол между двумя векторами = отношению произведения их координат к произведению их длин

            if (verify_angles(v, poly.v) == false)
                return false;

            return (verify_relate_edges(v, poly.v)) ? true : false;
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

Line* Polygon::plane_set_exterior() const {
    Line* line = new Line[v.size()];
    Line* return_line = line;
    //Take a cross product to find headeness
    int flag = (v[0].x - v[1].x) * (v[1].y - v[2].y) > (v[0].y - v[1].y) * (v[1].x - v[2].x);

    for (int p1 = v.size() - 1, p2 = 0; p2 < v.size(); p1 = p2, ++p2, ++line) {
        line->a = v[p1].y - v[p2].y;
        line->b = v[p2].x - v[p1].x;
        line->c = line->a * v[p1].x + line->b * v[p1].y;

        if (flag) {
            line->a = -line->a;
            line->b = -line->b;
            line->c = -line->c;
        }
    }

    line = return_line;
    //Randomize order of edges increase chance of early out
    for (int i = 0; i < v.size(); ++i) {
        double rd = ((double)rand() / (double)RAND_MAX);
        int ind = (int)(rd * v.size());;
        if (ind < 0 || ind >= v.size())
            ind = 0;
        Line temp_line = *line;
        *line = return_line[ind];
        return_line[ind] = temp_line;
    }
    return return_line;
}

bool Polygon::exterior_test(const Point& point) const {
    Line* line = plane_set_exterior();
    Line* delete_line = line;
    for (int i = 0; i < v.size(); ++i, ++line) {
        if (line->a * point.x + line->b * point.y > line->c) {
            delete[] delete_line;
            return false;
        }
    }
    delete[] delete_line;
    return true;
}

void Polygon::helper_crossing_test(
    const Point& point, const Point& p_0,
    const Point& p_1, bool& xflag_0,
    bool& yflag_0, double& crossing_numbers) const {
    xflag_0 = (p_0.x >= point.x);
    // verify if vertex are on same side of Y axis( X signs is same)
    if (xflag_0 == (p_1.x >= point.x)) {
        // if both point right then ray intersect polygon side
        if (xflag_0) {
            crossing_numbers += (yflag_0) ? -1 : 1;
        }
    }
    else {
        // compute intersection point polygon side with X axis

        //if ((p_1.x - (p_1.y - point.y) * (p_0.x - p_1.x) / (p_0.y - p_1.y)) >= point.x) {
        //    crossing_numbers += (yflag_0) ? -1 : 1;
        //}

        Point new_p0(p_0.x - point.x, p_0.y - point.y);
        Point new_p1(p_1.x - point.x, p_1.y - point.y);
        Point new_direct_vector = direct_vector(new_p0, new_p1);
        if ((new_direct_vector.y * new_p0.x - new_direct_vector.x * new_p0.y) / new_direct_vector.y >= 0)
            crossing_numbers += (yflag_0) ? -1 : 1;
    }
}

bool Polygon::crossing_test_not_convex(const Point& point) const {
    Point p_0, p_1;
    bool xflag_0, yflag_0, yflag_1;
    bool inside_flag = false;
    double crossing_numbers = 0;

    p_0 = v.back();
    yflag_0 = (p_0.y >= point.y);
    for (size_t i = 0; i < v.size(); ++i) {
        if (point == v[i])
            return true;
        p_1 = v[i];
        yflag_1 = (p_1.y >= point.y);
        // verify if vertex straddle (on opposite side) of X asis ( Y signs is differ)
        if (yflag_0 != yflag_1) {  // side of polygon can intersect ray shoted from testing point
            helper_crossing_test(point, p_0, p_1, xflag_0, yflag_0, crossing_numbers);
        }
        p_0 = p_1;
        yflag_0 = yflag_1;
    }
    inside_flag = (crossing_numbers != 0);
    return inside_flag;
}

bool Polygon::crossing_test_convex(const Point& point) const {
    Point p_0, p_1;
    bool xflag_0, yflag_0, yflag_1;
    bool inside_flag = false;
    double crossing_numbers = 0;
    bool convex_flag = false;

    p_0 = v.back();
    yflag_0 = (p_0.y >= point.y);
    for (size_t i = 0; i < v.size(); ++i) {
        if (point == v[i])
            return true;
        p_1 = v[i];
        yflag_1 = (p_1.y >= point.y);
        // verify if vertex straddle (on opposite side) of X asis ( Y signs is differ)
        if (yflag_0 != yflag_1) {  // side of polygon can intersect ray shoted from testing point
            helper_crossing_test(point, p_0, p_1, xflag_0, yflag_0, crossing_numbers);
            if (convex_flag) {
                break;
                convex_flag = true;
            }
        }
        p_0 = p_1;
        yflag_0 = yflag_1;
    }
    inside_flag = (crossing_numbers != 0);
    return inside_flag;
}

int compare_sl(const void* sl, const void* sl2) {
    const Polygon::Size_plane_not_convex* sp_1 = static_cast<const Polygon::Size_plane_not_convex*>(sl);
    const Polygon::Size_plane_not_convex* sp_2 = static_cast<const Polygon::Size_plane_not_convex*>(sl2);

    if (sp_1->area == sp_2->area)
        return 0;
    return (sp_1->area < sp_2->area) ? -1 : 1;
}

Line* Polygon::plane_set_not_convex() const {
    Line* line = new Line[(v.size() - 2) * 3];
    Size_plane_not_convex* sl = new Size_plane_not_convex[v.size() - 2];
    double len[3];
    Line* return_line = line;

    double v0x = v.front().x;
    double v0y = v.front().y;

    for (int p1 = 1, p2 = 2; p2 < v.size(); ++p1, ++p2) {
        line->a = v0y - v[p1].y;
        line->b = v[p1].x - v0x;
        line->c = line->a * v0x + line->b * v0y;

        len[0] = line->a * line->a + line->b * line->b;

        sl[p1 - 1].ptr = line;
        sl[p1 - 1].area = (
            (v0x - v[p1].y) +
            (v[p1].x - v[p2].y) +
            (v[p2].x - v0y) -
            (v[p1].x - v0y) -
            (v[p2].x - v[p1].y) -
            (v0x - v[p2].y)
            );
        ++line;

        line->a = v[p1].y - v[p2].y;
        line->b = v[p2].x - v[p1].x;
        line->c = line->a * v[p1].x + line->b * v[p1].y;

        len[1] = line->a * line->a + line->b * line->b;

        ++line;
        line->a = v[p2].y - v0y;
        line->b = v0x - v[p2].x;
        line->c = line->a * v[p2].x + line->b * v[p2].y;

        len[2] = line->a * line->a + line->b * line->b;

        double tx = (v0x * v[p1].x * v[p2].x) / 3;
        double ty = (v0y * v[p1].y * v[p2].y) / 3;

        if (line->a * tx + line->b * ty >= line->c) {
            line -= 2;
            for (int i = 0; i < 3; ++i) {
                line->a = -line->a;
                line->b = -line->b;
                line->c = -line->c;
                ++line;
            }
        }
        else {
            ++line;
        }
        line -= 3;
        for (int i = 0; i < 2; ++i)
            for (int j = i + 1; j < 3; ++j) {
                if (len[i] < len[j]) {
                    double temp_len = len[i];
                    len[i] = len[j];
                    len[j] = temp_len;
                    Line temp_line = line[i];
                    line[i] = line[j];
                    line[j] = temp_line;
                }
            }
        line += 3;
    }
    std::qsort(sl, v.size() - 2, sizeof(Size_plane_not_convex), &compare_sl);

    line = return_line;

    for (int i = 0; i < v.size() - 2; ++i) {
        Line* new_line = sl[i].ptr;
        if (new_line < line)
            continue;
        for (int j = 0; j < 3; ++j, ++line, ++new_line) {
            Line temp_set = *line;
            *line = *new_line;
            *new_line = temp_set;
        }
    }

    delete[] sl;
    return return_line;
}

bool Polygon::triangle_test_not_convex(const Point& point) const {
    Line* line = plane_set_not_convex();
    Line* delete_line = line;
    bool inside_flag = false;

    for (int i = 0; i < v.size() - 2; ++i) {

        if (line->a * point.x + line->b * point.y <= line->c) {
            if (line->a * point.x + line->b * point.y == line->c)
                return true;
            ++line;
            if (line->a * point.x + line->b * point.y <= line->c) {
                if (line->a * point.x + line->b * point.y == line->c)
                    return true;
                ++line;
                if (line->a * point.x + line->b * point.y <= line->c) {
                    inside_flag = !inside_flag;
                }
                ++line;
            }
            else {
                line += 2;
            }
        }
        else {
            line += 3;
        }
    }
    delete[] delete_line;
    return inside_flag;
}

Polygon::Plane_set* Polygon::plane_set_convex() const {
    Plane_set* ps = new Plane_set[(v.size() - 2) * 3];
    Size_plane* sp = new Size_plane[v.size() - 2];
    double len[3];
    Plane_set* return_ps = ps;

    double v0x = v.front().x;
    double v0y = v.front().y;

    for (int p1 = 1, p2 = 2; p2 < v.size(); ++p1, ++p2) {
        ps->line.a = v0y - v[p1].y;
        ps->line.b = v[p1].x - v0x;
        ps->line.c = ps->line.a * v0x + ps->line.b * v0y;

        len[0] = ps->line.a * ps->line.a + ps->line.b * ps->line.b;
        ps->ext_flag = (p1 == 1);

        sp[p1 - 1].ptr = ps;
        sp[p1 - 1].area = (
            (v0x - v[p1].y) +
            (v[p1].x - v[p2].y) +
            (v[p2].x - v0y) -
            (v[p1].x - v0y) -
            (v[p2].x - v[p1].y) -
            (v0x - v[p2].y)
            );

        ++ps;
        ps->line.a = v[p1].y - v[p2].y;
        ps->line.b = v[p2].x - v[p1].x;
        ps->line.c = ps->line.a * v[p1].x + ps->line.b * v[p1].y;

        len[1] = ps->line.a * ps->line.a + ps->line.b * ps->line.b;
        ps->ext_flag = true;

        ++ps;
        ps->line.a = v[p2].y - v0y;
        ps->line.b = v0x - v[p2].x;
        ps->line.c = ps->line.a * v[p2].x + ps->line.b * v[p2].y;

        len[2] = ps->line.a * ps->line.a + ps->line.b * ps->line.b;
        ps->ext_flag = (p2 == v.size() - 1);

        double tx = (v0x * v[p1].x * v[p2].x) / 3;
        double ty = (v0y * v[p1].y * v[p2].y) / 3;

        if (ps->line.a * tx + ps->line.b * ty >= ps->line.c) {
            ps -= 2;
            for (int i = 0; i < 3; ++i) {
                ps->line.a = -ps->line.a;
                ps->line.b = -ps->line.b;
                ps->line.c = -ps->line.c;
                ++ps;
            }
        }
        else {
            ++ps;
        }
        ps -= 3;
        for (int i = 0; i < 2; ++i)
            for (int j = i + 1; j < 3; ++j) {
                if (len[i] < len[j]) {
                    double temp_len = len[i];
                    len[i] = len[j];
                    len[j] = temp_len;
                    Plane_set temp_set = ps[i];
                    ps[i] = ps[j];
                    ps[j] = temp_set;
                }
            }
        ps += 3;
    }
    std::qsort(sp, v.size() - 2, sizeof(Size_plane), &compare_sp);

    ps = return_ps;
    for (int i = 0; i < v.size() - 2; ++i) {
        Plane_set* temp_set = sp[i].ptr;
        if (temp_set <= ps)
            continue;
        for (int j = 0; j < 3; ++j, ++ps, ++temp_set) {
            Plane_set temp = *ps;
            *ps = *temp_set;
            *temp_set = *ps;
        }
    }
    delete[] sp;
    return return_ps;
}

bool Polygon::triangle_test_convex(const Point& point) const {
    Plane_set* ps = plane_set_convex();
    Plane_set* delete_ps = ps;
    bool inside_flag = false;

    for (int i = 0; i < v.size() - 2; ++i) {
        if (ps->line.a * point.x + ps->line.b * point.y <= ps->line.c) {
            if (ps->line.a * point.x + ps->line.b * point.y == ps->line.c) {
                inside_flag = true;
                break;
            }
            ++ps;
            if (ps->line.a * point.x + ps->line.b * point.y <= ps->line.c) {
                if (ps->line.a * point.x + ps->line.b * point.y == ps->line.c) {
                    inside_flag = true;
                    break;
                }
                ++ps;
                if (ps->line.a * point.x + ps->line.b * point.y <= ps->line.c) {
                    inside_flag = true;
                    break;
                }
                else if (ps->ext_flag) {
                    inside_flag = false;
                    break;
                }
                ++ps;
            }
            else {
                if (ps->ext_flag) {
                    inside_flag = false;
                    break;
                }
                ps += 2;
            }
        }
        else {
            if (ps->ext_flag) {
                inside_flag = false;
                break;
            }
            ps += 3;
        }
    }
    delete[] delete_ps;
    return inside_flag;
}

bool Polygon::contains_point(const Point& point) const {
    if (is_convex_optimized()) {
        if (v.size() < 10)
            return (area_bounding_box() / area() > 2) ? exterior_test(point) : triangle_test_convex(point);
        else
            return crossing_test_convex(point);
    }
    else {
        return (v.size() < 10) ? triangle_test_not_convex(point) : crossing_test_not_convex(point);
    }
}

void Polygon::rotate(const Point& point, double angle) {
    for (size_t i = 0; i < v.size(); ++i) {
        v[i].rotate(point, angle);
    }
}

void Polygon::reflect_by_point(const Point& point) {
    for (size_t i = 0; i < v.size(); ++i) {
        v[i].reflect_by_point(point);
    }
}

void Polygon::reflect_by_line(const Line& line) {
    for (size_t i = 0; i < v.size(); ++i) {
        v[i].reflect_by_line(line);
    }
}

void Polygon::scale(const Point& center, double coeff) {
    for (size_t i = 0; i < v.size(); ++i) {
        v[i].homotetia(center, coeff);
    }
}

double Polygon::area_bounding_box() const {
    Point x_min = v.front();
    Point y_max = v.front();
    Point x_max = v.front();
    Point y_min = v.front();
    for (int i = 1; i < v.size(); ++i) {
        if (v[i].x < x_min.x)
            x_min = v[i];
        if (v[i].y > y_max.y)
            y_max = v[i];
        if (v[i].x > x_max.x)
            x_max = v[i];
        if (v[i].y < y_min.y)
            y_min = v[i];
    }
    Line line_1(x_min, Point(x_min.x, 0.0));
    Line line_2(y_max, Point(0, y_max.y));
    Line line_3(x_max, Point(x_max.x, 0));
    Line line_4(y_min, Point(0, y_min.y));
    Point p_1 = intersection(line_1, line_2);
    Point p_2 = intersection(line_2, line_3);
    Point p_3 = intersection(line_3, line_4);
    double d_1 = dist(p_1, p_2);
    double d_2 = dist(p_2, p_3);
    return d_1 * d_2;
}