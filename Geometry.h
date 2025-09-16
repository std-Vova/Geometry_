#include <vector>
#include <initializer_list>
#include <utility>
#include <iostream>
class Line;

struct Point {
    double x;
    double y;

    Point();
    Point(double, double);
    Point(const Point&);
    Point(const Point&, const Point&);

    Point& operator=(const Point&);

    Point& operator-=(const Point&);
    friend Point operator-(const Point&, const Point&);
    Point& operator+=(const Point&);
    friend Point operator+(const Point&, const Point&);

    bool operator==(const Point&) const;
    bool operator<(const Point&) const;
    bool operator>(const Point&) const;

    std::partial_ordering operator<=>(const Point&) const;

    Point rotate(const Point&, double) const;
    void reflect_by_point(const Point&);
    void reflect_by_line(const Line&);
    void homotetia(const Point&, double);

    Point& operator*=(double);
    friend Point operator*(const Point&, double);
};

class Line {
public:       // public for tests
    double a;
    double b;
    double c;

public:
    Line();
    Line(Point, Point);
    Line(double, double);
    Line(Point, double);
    Line(double, double, double);

    bool operator==(const Line&) const;
    bool operator!=(const Line&) const;

};

Line orthogonal(const Point&, const Point&);

class Shape {
public:
    virtual double perimeter() const = 0;
    virtual double area() const = 0;
    virtual bool is_congruent_to(const Shape&) const = 0;
    virtual bool is_similar_to(const Shape&) const = 0;
    virtual bool contains_point(const Point&) const = 0;
    virtual void rotate(const Point&, double) = 0;
    virtual void reflect_by_point(const Point&) = 0;
    virtual void reflect_by_line(const Line&) = 0;
    virtual void scale(const Point&, double) = 0;

    bool operator==(const Shape&) const;
};

class Polygon : public Shape{
protected:
    std::vector<Point> v;
public:
    enum struct PolygonClass {
        ConvexCW = 1,
        ConvexCCW = 2,
        DegenerateConvex = 3,  // все точки могут лежать на одной линии и удовлетворять условию 4
        NotConvexDegenerate = 4, //  все точки могут лежать на одной линии и не удовлетворять условию 4
        NotConvex = 5
    };

public:
    Polygon() = default;
    Polygon(const std::vector<Point>&);
    Polygon(const std::initializer_list<Point>&);

private:
    Polygon(size_t);

public:
    int vertices_count() const;
    const std::vector<Point>& get_vertices() const;
    bool is_convex() const;
    PolygonClass type_polygon() const;
    bool is_convex_optimized() const;
    PolygonClass type_polygon_optimized() const;

    double perimeter() const override;
    double area() const override;
    bool is_congruent_to(const Shape&) const override;
    bool is_similar_to(const Shape&) const override;
    bool contains_point(const Point&) const override;
    void rotate(const Point&, double) override;
    void reflect_by_point(const Point&) override;
    void reflect_by_line(const Line&) override;
    void scale(const Point&, double) override;

private:
    int check_side(Point, Point, Point) const;
    int compare(Point, Point) const;

    int check_turn_optimized(Point, Point) const;
    int compare_optimazed(Point) const;
    bool check_conditional_and_shift_optimize(Point&, Point&, int&, int&, int&, int&, int&) const;

    template <typename T>
    bool check_conditional_and_shift(const T&) const;
public:
    struct Plane_set {
        Line line;
        bool ext_flag = true;
    };

    struct Size_plane {
        Plane_set* ptr = nullptr;
        double area = 0;
    };
    
    struct Size_plane_not_convex {
        Line* ptr = nullptr;
        double area = 0;
    };
    double area_bounding_box() const;
private:
    Plane_set* plane_set_convex() const;
    Line* plane_set_not_convex() const;
    Line* plane_set_exterior() const;
    bool exterior_test(const Point&) const;
    bool crossing_test_convex(const Point&) const;
    bool crossing_test_not_convex(const Point&) const;
    void helper_crossing_test(const Point&, const Point&,
       const Point&, bool&,
        bool&, double&) const;
    bool triangle_test_not_convex(const Point&) const;

protected:
    bool triangle_test_convex(const Point&) const;
};

class Rectangle : public Polygon {
public:
    Rectangle() = default;
    Rectangle(const Point&, const Point&, double k);  // обход прямоугольника по часовой стрелке, центр никода не может быть меньше наименьшей точки
    using Polygon::Polygon;
public:
    Point center() const;
    std::pair<Line, Line> diagonals() const;

    double perimeter() const override;
    double area() const override;
    bool is_congruent_to(const Shape&) const override;
    bool is_similar_to(const Shape&) const override;
};

class Elipsoid : public Shape {
    Point focus_left;
    Point focus_right;
    double dist;

public:
    Elipsoid() = default;
    Elipsoid(const Point&, const Point&, double);

public:
    std::pair<const Point&, const Point&> focuses() const;
    std::pair<Line, Line> directrices() const;
    double eccentricity() const;
    Point center() const;

    double perimeter() const override;
    double area() const override;
    bool is_congruent_to(const Shape&) const override;
    bool is_similar_to(const Shape&) const override;
    bool contains_point(const Point&) const override;
    void rotate(const Point&, double) override;
    void reflect_by_point(const Point&) override;
    void reflect_by_line(const Line&) override;
    void scale(const Point&, double) override;
};

class Circle : public Elipsoid {
public:
    Point center;
    double radius = 0;

public:
    Circle() = default;
    Circle(const Point&, double);

    bool operator==(const Circle&) const;

public:
    double perimeter() const override;
    double area() const override;
    bool is_congruent_to(const Shape&) const override;
    bool is_similar_to(const Shape&) const override;
    bool contains_point(const Point&) const override;
    void rotate(const Point&, double) override;
    void reflect_by_point(const Point&) override;
    void reflect_by_line(const Line&) override;
    void scale(const Point&, double) override;
};

class Square : public Rectangle {
public:
    Square() = default;
    Square(const Point&, const Point&);
    using Rectangle::Rectangle;
public:
    Circle circumscribed_circle() const;
    Circle inscribed_circle() const;

public:
    double perimeter() const override;
    bool is_congruent_to(const Shape&) const override;
    bool is_similar_to(const Shape&) const override;
};

class Triangle : public Polygon {
public:
    using Polygon::Polygon;

    Circle circumscribed_circle() const;
    Circle inscribed_circle() const;
    Point centroid() const;
    Point ortho_center() const;
    Line euler_line() const;
    Circle nine_points_circle() const;

public:
    double perimeter() const override;
    double area() const override;
    bool is_congruent_to(const Shape&) const override;
    bool is_similar_to(const Shape&) const override;
    bool contains_point(const Point&) const override;
};

const double PI = 3.141592653589793;
double round_(double);
Point intersection(const Line&, const Line&);
double dist(const Point&, const Point&);
Point direct_vector(const Point&, const Point&);
Line orthogonal(const Point&, const Point&);
Line orthogonal_to_line(const Point&, const Line&);
double angles(const Point& , const Point&, const Point&);
bool verify_angles(const std::vector<Point>&, const std::vector<Point>&);
bool verify_edges(const std::vector<Point>&, const std::vector<Point>&);
bool verify_relate_edges(const std::vector<Point>&, const std::vector<Point>&);
int compare_sp(const void*, const void*);