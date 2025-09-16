#include "pch.h"
#include "Geometry.h"

// Point 

TEST(correctness, operator_comparison_point) {
	Point test(5, 5);

	Point x1(5, 5);
	Point x2(5, 8);
	Point x3(5, 3);
	Point x4(1, 5);
	Point x5(7, 5);
	
		EXPECT_TRUE(test == x1);
		EXPECT_TRUE(test < x2);
		EXPECT_TRUE(test > x3);
		EXPECT_TRUE(test >= x1);
		EXPECT_TRUE(test <= x2);
		EXPECT_TRUE(test > x4);
		EXPECT_TRUE(test < x5);

		EXPECT_FALSE(test != x1);
		EXPECT_FALSE(test >= x2);
		EXPECT_FALSE(test == x3);
		EXPECT_FALSE(test < x4);
		EXPECT_FALSE(test >= x5);

		EXPECT_FALSE(Point(5, 9) < test);
		EXPECT_TRUE(Point(9, 5) > test);
	
}

// Line 

TEST(correctness, operator_equal_Line) {
	Line line1(Point(1, 1), Point(2, 2));
	Line line2(Point(3, 3), Point(4, 4));
	Line line3(Point(5, 5), Point(4, 6));

	EXPECT_TRUE(line1 == line2);
	EXPECT_FALSE(line1 == line3);


}

// Polygon 
TEST(correctness, is_convex) {
	Polygon p = { Point(1,1), Point(2,2) }; // Convex Degenerate 
	Polygon p2 = { Point(1,1), Point(1,3), Point(4,3), Point(3,3), Point(5,3), Point(5,1) }; // Not Convex
	Polygon p3 = { Point(1,1), Point(1,4), Point(4,4), Point(4,2), Point(2,2), Point(2,3), Point(3,3), Point(3,1) }; // Not Convex
	Polygon p4 = { Point(1,1), Point(2,3), Point(2,2), Point(3,3), Point(5,1) }; // Not Convex
	Polygon p5 = { Point(1,0), Point(2,0), Point(3,0), Point(4,0), Point(2,0), Point(1,0) }; // Convex Degenerate
	Polygon p6 = { Point(1,2), Point(3,2), Point(6,2), Point(8,2), Point(5,2), Point(2,2), Point(10,2), Point(11,2), Point(9,2) }; // Not Convex Degenerate
	Polygon p7 = { Point(0,3), Point(3,3), Point(1,1), Point(2,4), Point(4,1) }; // Star 
	Polygon p8 = { Point(1,2), Point(2,3), Point(4,3), Point(4,2), Point(2,1) }; //  ConvexCCW
	Polygon p9 = { Point(1,2), Point(1,2), Point(1,2),Point(2,3), Point(4,3), Point(4,2), Point(2,1) }; // ConvexCCW
	Polygon p10 = { Point(1, 2), Point(2, 3), Point(4, 3), Point(4, 2), Point(2, 1) ,Point(2,1), Point(2,1), Point(2,1) }; // ConvexCCW
	Polygon p11 = { Point(1, 2), Point(2, 3), Point(4, 3), Point(4, 2), Point(1, 2) }; // ConvexCCW
	Polygon p12 = { Point(1, 2), Point(1, 2), Point(1, 2), Point(4, 3), Point(4, 2), Point(1, 2) }; // ConvexCCW

	EXPECT_TRUE(p.is_convex());
	EXPECT_EQ(p.type_polygon(), Polygon::PolygonClass::DegenerateConvex);

	EXPECT_FALSE(p2.is_convex());
	EXPECT_EQ(p2.type_polygon(), Polygon::PolygonClass::NotConvex);

	EXPECT_FALSE(p3.is_convex());
	EXPECT_EQ(p3.type_polygon(), Polygon::PolygonClass::NotConvex);

	EXPECT_FALSE(p4.is_convex());
	EXPECT_EQ(p4.type_polygon(), Polygon::PolygonClass::NotConvex);


	EXPECT_TRUE(p5.is_convex());
	EXPECT_EQ(p5.type_polygon(), Polygon::PolygonClass::DegenerateConvex);

	EXPECT_FALSE(p6.is_convex());
	EXPECT_EQ(p6.type_polygon(), Polygon::PolygonClass::NotConvexDegenerate);

	EXPECT_FALSE(p7.is_convex());  // star not convex 
	EXPECT_EQ(p7.type_polygon(), Polygon::PolygonClass::NotConvex);

	EXPECT_TRUE(p8.is_convex());
	EXPECT_EQ(p8.type_polygon(), Polygon::PolygonClass::ConvexCW);

	EXPECT_TRUE(p9.is_convex());
	EXPECT_EQ(p9.type_polygon(), Polygon::PolygonClass::ConvexCW);

	EXPECT_TRUE(p10.is_convex());
	EXPECT_EQ(p10.type_polygon(), Polygon::PolygonClass::ConvexCW);

	EXPECT_TRUE(p11.is_convex());
	EXPECT_EQ(p11.type_polygon(), Polygon::PolygonClass::ConvexCW);

	EXPECT_TRUE(p12.is_convex());
	EXPECT_EQ(p12.type_polygon(), Polygon::PolygonClass::ConvexCW);
}

TEST(correctness, is_convex_optimized) {
	
	Polygon p = { Point(1,1), Point(2,2) }; // Convex Degenerate 
	Polygon p2 = { Point(1,1), Point(1,3), Point(4,3), Point(3,3), Point(5,3), Point(5,1) }; // Not Convex
	Polygon p3 = { Point(1,1), Point(1,4), Point(4,4), Point(4,2), Point(2,2), Point(2,3), Point(3,3), Point(3,1) }; // Not Convex
	Polygon p4 = { Point(1,1), Point(2,3), Point(2,2), Point(3,3), Point(5,1) }; // Not Convex
	Polygon p5 = { Point(1,0), Point(2,0), Point(3,0), Point(4,0), Point(2,0), Point(1,0) }; // Convex Degenerate
	Polygon p6 = { Point(1,2), Point(3,2), Point(6,2), Point(8,2), Point(5,2), Point(2,2), Point(10,2), Point(11,2), Point(9,2) }; // Not Convex Degenerate
	Polygon p7 = { Point(0,3), Point(3,3), Point(1,1), Point(2,4), Point(4,1) }; // Star 
	Polygon p8 = { Point(1,2), Point(2,3), Point(4,3), Point(4,2), Point(2,1) }; //  ConvexCCW
	Polygon p9 = { Point(1,2), Point(1,2), Point(1,2),Point(2,3), Point(4,3), Point(4,2), Point(2,1) }; // ConvexCCW
	Polygon p10 = { Point(1, 2), Point(2, 3), Point(4, 3), Point(4, 2), Point(2, 1) ,Point(2,1), Point(2,1), Point(2,1)}; // ConvexCCW
	Polygon p11 = { Point(1, 2), Point(2, 3), Point(4, 3), Point(4, 2), Point(1, 2) }; // ConvexCCW
	Polygon p12 = { Point(1, 2), Point(1, 2), Point(1, 2), Point(4, 3), Point(4, 2), Point(1, 2) }; // ConvexCCW
	
	EXPECT_TRUE(p.is_convex_optimized());
	EXPECT_EQ(p.type_polygon_optimized(), Polygon::PolygonClass::DegenerateConvex);

	EXPECT_FALSE(p2.is_convex_optimized());
	EXPECT_EQ(p2.type_polygon_optimized(), Polygon::PolygonClass::NotConvex);

	EXPECT_FALSE(p3.is_convex_optimized());
	EXPECT_EQ(p3.type_polygon_optimized(), Polygon::PolygonClass::NotConvex);

	EXPECT_FALSE(p4.is_convex_optimized());
	EXPECT_EQ(p4.type_polygon_optimized(), Polygon::PolygonClass::NotConvex);

	EXPECT_TRUE(p5.is_convex_optimized());
	EXPECT_EQ(p5.type_polygon_optimized(), Polygon::PolygonClass::DegenerateConvex);
	
	EXPECT_FALSE(p6.is_convex_optimized());
	EXPECT_EQ(p6.type_polygon_optimized(), Polygon::PolygonClass::NotConvexDegenerate);
	
	EXPECT_FALSE(p7.is_convex_optimized());  // star not convex 
	EXPECT_EQ(p7.type_polygon_optimized(), Polygon::PolygonClass::NotConvex);

	EXPECT_TRUE(p8.is_convex_optimized());
	EXPECT_EQ(p8.type_polygon_optimized(), Polygon::PolygonClass::ConvexCW);

	EXPECT_TRUE(p9.is_convex_optimized());
	EXPECT_EQ(p9.type_polygon_optimized(), Polygon::PolygonClass::ConvexCW);

	EXPECT_TRUE(p10.is_convex_optimized());
	EXPECT_EQ(p10.type_polygon_optimized(), Polygon::PolygonClass::ConvexCW);

	EXPECT_TRUE(p11.is_convex_optimized());
	EXPECT_EQ(p11.type_polygon_optimized(), Polygon::PolygonClass::ConvexCW);

	EXPECT_TRUE(p12.is_convex_optimized());
	EXPECT_EQ(p12.type_polygon_optimized(), Polygon::PolygonClass::ConvexCW);
	
}

TEST(correctness, polygon_perimeter) {
	Polygon p4 = { Point(1,1), Point(2,3), Point(2,2), Point(3,3), Point(5,1) }; // Not Convex
	double p = p4.perimeter();
	EXPECT_NEAR(p, 11.4, 0.1);

	Polygon p8 = { Point(1,2), Point(2,3), Point(4,3), Point(4,2), Point(2,1) }; //  ConvexCCW
	double p2 = p8.perimeter();
	EXPECT_NEAR(p2, 8, 0.1);

	Polygon p7 = { Point(0,3), Point(3,3), Point(1,1), Point(2,4), Point(4,1) }; // Star 
	double p3 = p7.perimeter();
	EXPECT_NEAR(p3, 17.1, 0.1);
}

// ELIPSOID

TEST(correctness, elipsoid) {
	Elipsoid e(Point(-1.99295,1.2), Point(12.19295,1.2), 16);

	EXPECT_EQ(Point(5.1, 1.2), e.center());
	EXPECT_DOUBLE_EQ(e.eccentricity(), 0.88661875); // 0.88661875
	EXPECT_TRUE(Line(Point(8 / e.eccentricity(),1.2),Point(8 / e.eccentricity() + 5,1.2)) ==
		Line(Point(9.02304401, 1.2), Point(9.02304401 + 5, 1.2)));
}

TEST(correctness, elipsoid_perimeter) {
	Elipsoid l(Point(-4, 0), Point(4, 0), 10);
	double p = l.perimeter();
	//EXPECT_NEAR(p, 47.12, 0.1);
	EXPECT_NEAR(p, 25.1,0.1);
}

// RECTANGLE

TEST(correctness, rectangle) {
	
	Rectangle r(Point(1,0), Point(0,1), 1);
	EXPECT_EQ(r.center(), Point(0.5,0.5));
	auto pair = r.diagonals();
	EXPECT_NEAR(pair.second.a, Line(Point(0, 0), Point(1, 1)).a, 0.1);
	EXPECT_NEAR(pair.second.b, Line(Point(0, 0), Point(1, 1)).b, 0.1);
	EXPECT_NEAR(pair.second.c, Line(Point(0, 0), Point(1, 1)).c, 0.1);
	
	
	Rectangle r2(Point(33.8,11.9), Point(66.2, 98.1), 0.857142857);
	EXPECT_EQ(r2.center(), Point(50,55));
	auto pair2 = r2.diagonals();
	
	EXPECT_NEAR(pair2.second.a, Line(Point(9.88, 77.6), Point(90.115, 32.4)).a, 0.8);
	EXPECT_NEAR(pair2.second.b, Line(Point(9.88, 77.6), Point(90.115, 32.4)).b, 0.8);
	EXPECT_NEAR(pair2.second.c, Line(Point(9.88, 77.6), Point(90.115, 32.4)).c, 0.8);

}

TEST(correctness, rectangle_perimeter) {
	Rectangle r = { Point(1,1), Point(1,4), Point(6,4), Point(1,6) };
	double p = r.perimeter();
	EXPECT_EQ(p, 16);
}

// SQUARE

TEST(correctness, Square) {
	Square q(Point(2, 2), Point(5, 5));
	EXPECT_EQ(q.center(), Point(3.5, 3.5));
	auto pair = q.diagonals();

	EXPECT_EQ(pair.first, Line(Point(2, 2), Point(5, 5)));
	EXPECT_EQ(pair.second, Line(Point(2, 5), Point(5, 2)));

	
	EXPECT_EQ(q.inscribed_circle(), Circle(Point(3.5, 3.5), 1.5));
	EXPECT_EQ(q.circumscribed_circle(), Circle(Point(3.5, 3.5), 2.12132));
}

TEST(correctness, square_2) {
	Square s(Point(4, 1), Point(4, 5));
	EXPECT_EQ(s.center(), Point(4, 3));
	auto pair = s.diagonals();
	EXPECT_EQ(pair.first, Line(Point(4, 1), Point(4, 5)));
	EXPECT_EQ(pair.second, Line(Point(2, 3), Point(6, 3)));
	
	EXPECT_EQ(s.inscribed_circle(), Circle(Point(4, 3), 1.4));
	EXPECT_EQ(s.circumscribed_circle(), Circle(Point(4, 3), 2));
}

//	TRIANGLE

TEST(correctness, triangle_circumscribed_circle) {
	Triangle a = { Point(9,6), Point(7,1), Point(2,2) };
	Circle c = a.circumscribed_circle();
	Circle temp(Point(5.12963, 4.64815), 4.09967);
	EXPECT_NEAR(c.center.x, temp.center.x,0.1);
	EXPECT_NEAR(c.center.y, temp.center.y,0.1);
	EXPECT_NEAR(c.radius, temp.radius, 0.1);
}

TEST(correctness, triangle_inscribed_circle) {
	Triangle a = { Point(2,2), Point(5,6), Point(8,3) };
	Circle c = a.inscribed_circle();
	Circle temp(Point(5.16, 3.91), 1.37);
	EXPECT_NEAR(c.center.x , temp.center.x,0.1);
	EXPECT_NEAR(c.center.y , temp.center.y,0.1);
	EXPECT_NEAR(c.radius , temp.radius, 0.1);
}

TEST(correctness, triangle_centroid) {
	Triangle a = { Point(2,2), Point(5,6), Point(8,3) };
	Point c = a.centroid();
	Point temp(5.0, 3.67);
	EXPECT_NEAR(c.x , temp.x,0.1);
	EXPECT_NEAR(c.y , temp.y,0.1);
}

TEST(correctness, triangle_ortho_center) {
	Triangle a = { Point(2,2), Point(5,6), Point(8,3) };
	Point o = a.ortho_center();
	Point temp(5.14286, 5.14286);
	EXPECT_NEAR(o.x, temp.x, 0.1);
	EXPECT_NEAR(o.y, temp.y, 0.1);
}

TEST(correctness, triangle_euler_line) {
	Triangle a = { Point(2,2), Point(5,6), Point(8,3) };
	Line temp(Point(5.14, 3.9), Point(5.14286, 5.14286));
	Line e = a.euler_line();
	EXPECT_NEAR(e.a, temp.a, 0.1);
	EXPECT_NEAR(e.b, temp.b, 0.1);
	EXPECT_NEAR(e.c, temp.c, 0.1);
}

TEST(correctness, triangle_euler_circle) {
	Triangle a = { Point(2,2), Point(5,6), Point(8,3) };
	Circle circum = a.circumscribed_circle();
	Point orto = a.ortho_center();
	Point center_euler = Point(orto, circum.center);
	Circle temp(center_euler, circum.radius / 2);

	Circle e = a.nine_points_circle();

	EXPECT_EQ(e.center.x , temp.center.x);
	EXPECT_EQ(e.center.y , temp.center.y);
	EXPECT_EQ(e.radius, temp.radius);
}

TEST(correctness, shape_perimeter) {
	Polygon p7 = { Point(0,3), Point(3,3), Point(1,1), Point(2,4), Point(4,1) }; // Star 
	Shape& shape = p7;
	double p1 = shape.perimeter();
	EXPECT_EQ(p1 , 17.1);

	Elipsoid l(Point(-4, 0), Point(4, 0), 10);
	Shape& shape2 = l;
	double p2 = shape2.perimeter();
	EXPECT_NEAR(p2, 25.13, 0.1);

	Rectangle r = { Point(1,1), Point(1,4), Point(6,4), Point(1,6) };
	Shape& shape3 = r;
	double p3 = shape3.perimeter();
	EXPECT_EQ(p3, 16);

	Circle c(Point(5, 5), 4);
	Shape& shape4 = c;
	double p4 = shape4.perimeter();
	EXPECT_NEAR(p4, 25.13, 0.1);
}

TEST(correctness, shape_area) {
	Polygon p7 = { Point(0,3), Point(3,3), Point(1,1), Point(2,4), Point(4,1) }; // Star 
	Shape& shape = p7;
	double p1 = shape.area();
	EXPECT_NEAR(p1, 4.5, 0.1);

	Elipsoid l(Point(-4, 0), Point(4, 0), 10);
	Shape& shape2 = l;
	double p2 = shape2.area();
	EXPECT_NEAR(p2, 47.1, 0.1);

	Rectangle r = { Point(1,1), Point(1,4), Point(6,4), Point(1,6) };
	Shape& shape3 = r;
	double p3 = shape3.area();
	EXPECT_EQ(p3, 15);

	Circle c(Point(5, 5), 4);
	Shape& shape4 = c;
	double p4 = shape4.area();
	EXPECT_NEAR(p4, 50.2, 0.1);
}

TEST(correctness, shape_coincide) {
	Circle c;
	Circle c2;
	Square s;
	Shape& shape_c = c;
	Shape& shape_s = s;
	Shape& shape_c2 = c2;
	EXPECT_TRUE(shape_c2 == shape_c);
	EXPECT_TRUE(c == shape_c2);
	EXPECT_FALSE(shape_c == shape_s);
	EXPECT_FALSE(s == shape_c2);
}

TEST(correctness, shape_is_congruent_to) {
	Polygon p1 = { Point(1,2), Point(2,3), Point(4,3), Point(4,2), Point(2,1) }; //  ConvexCCW
	Polygon p2 = { Point(1,2), Point(1,2), Point(1,2),Point(2,3), Point(4,3), Point(4,2), Point(2,1) , Point(5,6)};
	Polygon p3 = p1;
	Polygon* p4 = nullptr;
	Shape& sp1 = p1;
	Shape& sp2 = p2;
	Shape& sp3 = p3;
	Shape& sp4 = *p4;
	EXPECT_TRUE(p1.is_congruent_to(sp3));
	EXPECT_TRUE(sp1.is_congruent_to(sp3));
	EXPECT_FALSE(sp1.is_congruent_to(sp2));
	
	
	Circle c;
	Shape& sc = c;
	Polygon poly;
	Shape& poly_sc = poly;
	EXPECT_FALSE(sc.is_congruent_to(poly_sc));

	Square* ptr_sqr = new Square();
	Shape& shape_sqr = *ptr_sqr;
	Circle circl;
	Shape& shape_circl = circl;
	EXPECT_FALSE(shape_sqr.is_congruent_to(shape_circl), std::bad_typeid);
	
}

TEST(correctness, is_congruent_to_polygon) {
	Polygon p = { Point(-2,1), Point(-2,6), Point(8,6), Point(8,1) };
	Shape& ps = p;
	Polygon p2 = { Point(-2,1), Point(-5.53553391,4.53553391), Point(1.53553391,11.6066017), Point(5.07106781,8.07106781) };
	Shape& ps2 = p2;
	EXPECT_TRUE(ps.is_congruent_to(ps2));
}

TEST(correctness, is_congruent_to_rectangle) {
	Rectangle r = { Point(2,1), Point(2,4), Point(8,4), Point(8,1) };
	Shape& pr = r;
	Rectangle r2 = { Point(2,5), Point(2,8), Point(8,8), Point(8,5) };
	Shape& pr2 = r2;
	EXPECT_TRUE(pr.is_congruent_to(pr2));
}

TEST(correctness, is_congruent_to_square) {
	Square s = { Point(2,1), Point(2,4), Point(5,4), Point(5,1) };
	Polygon& p = s;
	Square s2 = { Point(2,5), Point(2,8), Point(5,8), Point(5,4) };
	Shape& sh = s2;
	EXPECT_TRUE(p.is_congruent_to(sh));
}

TEST(correctness, is_congruent_to_triangle) {
	Triangle t = { Point(1,1), Point(1,4), Point(5,1) };
	Polygon& poly = t;
	Triangle t2 = { Point(5,5), Point(5,8), Point(9,5) };
	Shape& s = t2;
	EXPECT_TRUE(poly.is_congruent_to(s));
}

TEST(correctness, is_similar_to_polygon) {
	Polygon p = { Point(1,1), Point(1,4), Point(5,4), Point(7,1) };
	Polygon p2 = { Point(1,1), Point(1,2.5), Point(3,2.5), Point(4,1) };
	Shape& s = p2;
	EXPECT_TRUE(p.is_similar_to(s));
}

TEST(correctness, is_similar_to_rectangle) {
	Rectangle r = { Point(0,1), Point(0,5), Point(6,4), Point(6,1) };
	Shape& sr = r;
	Rectangle r2 = { Point(0,1), Point(0,3), Point(3,3), Point(3,1) };
	Shape& sr2 = r2;
	EXPECT_TRUE(sr2.is_similar_to(sr));
}

TEST(correctness, is_similar_to_square) {
	Square s = { Point(0,1), Point(0,5), Point(4,5), Point(4,1) };
	Shape& sh = s;
	Square s2 = { Point(0,0), Point(0,2), Point(2,2), Point(2,0) };
	Shape& sh2 = s2;
	EXPECT_TRUE(sh.is_similar_to(sh2));
}

TEST(correctness, is_similar_to_triangle) {
	Triangle t = { Point(0,1), Point(6,5), Point(6,1) };
	Polygon& poly = t;
	Triangle t2 = { Point(3,1), Point(6,3), Point(6,1) };
	Shape& s = t2;
	EXPECT_TRUE(s.is_similar_to(poly));
}

TEST(correctness, is_congruent_to_elipsoid) {
	Elipsoid el(Point(3, 3), Point(9, 3), 8);
	Elipsoid el2(Point(6, 6), Point(12, 6), 8);
	EXPECT_TRUE(el.is_congruent_to(el2));
}

TEST(correctness, is_similar_to_elipsoid) {
	Elipsoid el(Point(3, 3), Point(9, 3), 8);
	Elipsoid el2(Point(0, 3), Point(12, 3), 16);
	EXPECT_TRUE(el.is_similar_to(el2));
}

TEST(correctness, rotate_polygon) {
	Polygon p = { Point(-2,1), Point(-2,6), Point(4,6), Point(4,1) };
	p.rotate(Point(-2,1),45);
	EXPECT_TRUE(p.is_congruent_to(Polygon{ Point(-2, 1), Point(-5.5, 4.5), Point(-1.3, 8.7), Point(2.2, 5.2) }));
}

TEST(correctness, rotate_circle) {
	Circle c(Point(3, 3), 5);
	c.rotate(Point(0,0), 45);
	EXPECT_TRUE(c.is_congruent_to(Circle(Point(0, 4.2), 5)));
}

TEST(correctness, reflex_by_point) {
	Square s = { Point(3,2), Point(3,3), Point(4,3), Point(4,2) };
	Shape& shape = s;
	shape.reflect_by_point(Point(5, 2.5));
	Square test_square = { Point(6,2), Point(6,3), Point(7,3), Point(7,2) };
	EXPECT_TRUE(shape.is_congruent_to(test_square));
}

TEST(correctness, reflex_by_line) {
	Polygon p = { Point(1,1), Point(1,3), Point(3,3), Point(3,1) };
	Shape& shape = p;
	shape.reflect_by_line(Line(Point(4,1), Point(4,3)));
	Polygon test_poly = { Point(5,1), Point(5,3), Point(7,3), Point(7,1) };
	EXPECT_TRUE(shape.is_congruent_to(test_poly));
}

TEST(correctness, reflex_by_crossing_line) {
	Polygon p = { Point(1,1), Point(1,3), Point(3,3), Point(3,1) };
	Shape& shape = p;
	shape.reflect_by_line(Line(Point(2.5, 1), Point(2.5, 3)));
	Polygon test_poly = { Point(2,1), Point(2,3), Point(4,3), Point(4,1) };
	EXPECT_TRUE(shape.is_congruent_to(test_poly));
}

TEST(correctness, scale) {
	Square s = { Point(2,1), Point(2,2), Point(3,2), Point(3,1) };
	Shape& shape = s;
	Square test_square = { Point(4,3), Point(4,5), Point(6,5), Point(6,3) };
	shape.scale(Point(1, 0), 2);
	EXPECT_TRUE(shape.is_similar_to(test_square));
}

TEST(correctness, scale_2) {
	Elipsoid el(Point(1, 1), Point(2, 1), 2);
	el.scale(Point(0, 0), 2);
	Elipsoid test_el(Point(3, 3), Point(6, 3), 4);
	EXPECT_TRUE(el.is_similar_to(test_el));
}

TEST(correctness, triangle_not_convex_test) {
	Polygon p = { Point(2,1), Point(1,4), Point(5,8), Point(9,2), Point(5,4) };
	Point p_1(3, 3);
	Point p_2(5, 8);
	Point p_3(5, 3);
	EXPECT_TRUE(p.contains_point(p_1));
	EXPECT_TRUE(p.contains_point(p_2));
	EXPECT_FALSE(p.contains_point(p_3));
}

TEST(correctness, triangle_convex_test) {
	Polygon p = { Point(1,3), Point(1,5), Point(3,6), Point(5,4), Point(3,2) };
	Point p_1(1, 3);
	Point p_2(2, 6);
	Point p_3(3, 4);
	EXPECT_TRUE(p.contains_point(p_1));
	EXPECT_FALSE(p.contains_point(p_2));
	EXPECT_TRUE(p.contains_point(p_3));
}

TEST(correctness, triangle_convex_test_2) {
	Triangle angle = { Point(1,1), Point(1,4),Point(4,1) };
	Point p_1(2, 2);
	Point p_2(4, 1);
	Point p_3(4, 5);
	EXPECT_TRUE(angle.contains_point(p_1));
	EXPECT_TRUE(angle.contains_point(p_2));
	EXPECT_FALSE(angle.contains_point(p_3));
}

TEST(correcntness, crossing_test_convex) {
	Polygon p = { Point(4,1), Point(3,2), Point(2,3), Point(2,5), Point(2.5,6.5), Point(4,7),
	Point(5,7), Point(8,6), Point(9,4), Point(7,1.5)};
	Point p_1(4, 1);
	Point p_2(5, 5);
	Point p_3(2, 2);
	EXPECT_TRUE(p.contains_point(p_1));
	EXPECT_TRUE(p.contains_point(p_2));
	EXPECT_FALSE(p.contains_point(p_3));
}

TEST(correctness, crossing_test_not_convex) {
	Polygon p = { Point(4,1), Point(3,2), Point(2,3), Point(2,5), Point(2.5,6.5), Point(4,7),
	Point(5,7), Point(8,6), Point(9,4), Point(6,5)};
	Point p_1(4, 1);
	Point p_2(5, 5);
	Point p_3(2, 2);
	EXPECT_TRUE(p.contains_point(p_1));
	EXPECT_TRUE(p.contains_point(p_2));
	EXPECT_FALSE(p.contains_point(p_3));
}

TEST(correctness, exterior_test) {
	Polygon p = { Point(3,1), Point(2,2), Point(6,5), Point(7,4) };
	Point p_1(3, 1);
	Point p_2(5, 3);
	Point p_3(6, 2);
	EXPECT_TRUE(p.contains_point(p_1));
	EXPECT_TRUE(p.contains_point(p_2));
	EXPECT_FALSE(p.contains_point(p_3));
}


