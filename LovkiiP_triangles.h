#ifndef TRIANGLES_INTERSECTION_LOVKIIP_TRIANGLES_H
#define TRIANGLES_INTERSECTION_LOVKIIP_TRIANGLES_H
#include<array>
#include<cmath>
#include<limits>
#include<iostream>
using namespace std;

namespace LovkiiP{
    class point3d;
    class segment3d;
    class triangle3d;
    class vector3d;

    struct plane{
        double A;
        double B;
        double C;
        double D;
    };

    class point3d{
    public:
        point3d(double x, double y, double z);
        bool is_equal(const point3d &p) const;
        bool is_on_plane(const plane &pln) const;
        bool in_triangle(triangle3d t);
        bool is_on_segment(segment3d s) const;
        double get_x() const { return x; };
        double get_y() const { return y; };
        double get_z() const { return z; };
    private:
        double x;
        double y;
        double z;
    };

    class triangle3d{
    public:
        triangle3d(point3d A, point3d B, point3d C);
        double area();
        bool is_intersect(const triangle3d &t);
        point3d get_A() { return A; };
        point3d get_B() { return B; };
        point3d get_C() { return C; };
    private:
        point3d A;
        point3d B;
        point3d C;
    };

    class segment3d{
    public:
        segment3d(point3d A, point3d B);
        double length();
        bool is_on_plane(const plane &pln);
        bool is_crossing(const plane &pln);
        bool is_crossing(segment3d s) ;
        bool is_crossing(triangle3d t);
        vector3d to_vector();
        point3d cross_point(const plane &pln);
        point3d get_begin() { return pbegin; };
        point3d get_end() { return pend; };
    private:
        point3d pbegin;
        point3d pend;
    };

    class vector3d{
    public:
        vector3d(double x, double y, double z, point3d p);
        double length() const;
        double scal_prod(const vector3d &v) const;
        vector3d vect_prod(const vector3d &v) const;
        double mixed_prod(const vector3d &v, const vector3d &u) const;
        bool is_collinear(const vector3d &v) const;
        double get_x() const { return x; };
        double get_y() const { return y; };
        double get_z() const { return z; };
        point3d get_p() { return p; };
    private:
        double x;
        double y;
        double z;
        point3d p;
    };

    plane get_plane(const point3d &p1, const point3d &p2, const point3d &p3);
    bool is_point(std::array<double, 9> &data);
    bool is_segment(std::array<double, 9> &data);
    bool is_triangle(std::array<double, 9> &data);
    point3d to_point(std::array<double, 9> &data);
    segment3d to_segment(std::array<double, 9> &data);
    triangle3d to_triangle(std::array<double, 9> &data);
    bool intersection(std::array<double, 9> &data1, std::array<double, 9> &data2);
}
#endif //TRIANGLES_INTERSECTION_LOVKIIP_TRIANGLES_H
