#include "LovkiiP_triangles.h"

using namespace std;
namespace LovkiiP{

//point3d methods implementation
    point3d::point3d(double x, double y, double z) : x(x), y(y), z(z) {}

    bool point3d::is_equal(const point3d &p) const {
        return ((std::fabs(this->x - p.x) < std::numeric_limits<double>::epsilon()) &&
                (std::fabs(this->y - p.y) < std::numeric_limits<double>::epsilon()) &&
                (std::fabs(this->z - p.z) < std::numeric_limits<double>::epsilon())
        );
    }

    bool point3d::is_on_plane(const plane &pln) const {
        return (std::fabs(pln.A * this->x + pln.B * this->y + pln.C * this->z + pln.D) <
                std::numeric_limits<double>::epsilon());
    }

    bool point3d::in_triangle(triangle3d t) {
        triangle3d ABP(t.get_A(), t.get_B(), *this);
        triangle3d BCP(t.get_B(), t.get_C(), *this);
        triangle3d CAP(t.get_C(), t.get_A(), *this);
        return (std::fabs(ABP.area() + BCP.area() + CAP.area() - t.area()) <
                                        std::numeric_limits<double>::epsilon());
    }

    bool point3d::is_on_segment(segment3d s) const {
        vector3d AP(this->x - s.get_begin().get_x(),
                    this->y - s.get_begin().get_y(),
                    this->z - s.get_begin().get_z(),
                    s.get_begin());
        vector3d BP(this->x - s.get_end().get_x(),
                    this->y - s.get_end().get_y(),
                    this->z - s.get_end().get_z(),
                    s.get_end());
        vector3d AB(s.get_end().get_x() - s.get_begin().get_x(),
                    s.get_end().get_y() - s.get_begin().get_y(),
                    s.get_end().get_z() - s.get_begin().get_z(),
                    s.get_begin());
        if (AB.is_collinear(AP)) {
            return (std::fabs(AP.length() + BP.length() - AB.length()) <
                                    std::numeric_limits<double>::epsilon());
        }
        return false;
    }

//triangle methods implementation
    triangle3d::triangle3d(point3d A, point3d B, point3d C) : A(A), B(B), C(C) {}

    double triangle3d::area() {
        segment3d a(this->A, this->B);
        segment3d b(this->B, this->C);
        segment3d c(this->C, this->A);
        double p = (a.length() + b.length() + c.length())/2;
        return sqrt(p * (p - a.length()) * (p - b.length()) * (p - c.length()));
    }

    bool triangle3d::is_intersect(const triangle3d &t) {
        segment3d AB(this->get_A(), this->get_B());
        segment3d BC(this->get_B(), this->get_C());
        segment3d CA(this->get_C(), this->get_A());
        return (AB.is_crossing(t) || BC.is_crossing(t) || CA.is_crossing(t));
    }

//segment methods implementation
    segment3d::segment3d(point3d A, point3d B) : pbegin(A), pend(B) {}

    double segment3d::length() {
        return (sqrt((this->pend.get_x() - this->pbegin.get_x()) * (this->pend.get_x() - this->pbegin.get_x()) +
                        (this->pend.get_y() - this->pbegin.get_y()) * (this->pend.get_y() - this->pbegin.get_y()) +
                        (this->pend.get_z() - this->pbegin.get_z()) * (this->pend.get_z() - this->pbegin.get_z())
                ));
    }

    bool segment3d::is_on_plane(const plane &pln) {
        return ((this->pbegin.is_on_plane(pln)) && this->pend.is_on_plane(pln));
    }

    bool segment3d::is_crossing(const plane &pln) {
        return ((pln.A * this->pbegin.get_x() + pln.B * this->pbegin.get_y() + pln.C * this->pbegin.get_z() + pln.D) *
                (pln.A * this->pend.get_x() + pln.B * this->pend.get_y() + pln.C * this->pend.get_z() + pln.D) <= 0);
    }

    bool segment3d::is_crossing(segment3d s) {
        plane pln = get_plane(s.get_begin(), s.get_end(), this->get_begin());
        if (!(this->get_end().is_on_plane(pln))) { return false; }
        vector3d v = this->to_vector();
        if (v.is_collinear(s.to_vector())) {
            return (this->get_begin().is_on_segment(s) || this->get_end().is_on_segment(s));
        }
        double x1 = this->get_begin().get_x();  double x3 = s.get_begin().get_x();
        double y1 = this->get_begin().get_y();  double y3 = s.get_begin().get_y();
        double x2 = this->get_end().get_x();    double x4 = s.get_end().get_x();
        double y2 = this->get_end().get_y();    double y4 = s.get_end().get_y();
        double u = ((x4 - x2) * (y4 - y3) - (y4 - y2) * (x4 - x3)) /
                ((x1 - x2) * (y4 - y3) - (y1 - y2) * (x4 - x3)); //possible division by zero

        double z1 = this->get_begin().get_z();
        double z2 = this->get_end().get_z();
        point3d p(u * (x1 - x2) + x2, u * (y1 - y2) + y2, u * (z1 - z2) + z2);
        return (p.is_on_segment(*this) && p.is_on_segment(s));
    }

    bool segment3d::is_crossing(triangle3d t) {
        segment3d AB(t.get_A(), t.get_B());
        segment3d BC(t.get_B(), t.get_C());
        segment3d CA(t.get_C(), t.get_A());
        plane pln = get_plane(t.get_A(), t.get_B(), t.get_C());
        if (this->is_on_plane(pln)) {
            return (this->is_crossing(AB) || this->is_crossing(BC) || this->is_crossing(CA) ||
                    this->get_begin().in_triangle(t) || this->get_end().in_triangle(t));
        }else{
            if (this->is_crossing(pln)){
                point3d cross = this->cross_point(pln);
                return cross.in_triangle(t);
            }
        }
        return false;
    }

    vector3d segment3d::to_vector() {
        vector3d ret(this->get_end().get_x() - this->get_begin().get_x(),
                  this->get_end().get_y() - this->get_begin().get_y(),
                  this->get_end().get_z() - this->get_begin().get_z(),
                  this->get_begin());
        return ret;
    }

    point3d segment3d::cross_point(const plane &pln) { // possible division by zero
        vector3d a = this->to_vector();
        double denominator = pln.A * a.get_x() + pln.B * a.get_y() + pln.C * a.get_z();
        double t = -(pln.D + pln.A * a.get_p().get_x() + pln.B *a.get_p().get_y() + pln.C *a.get_p().get_z()) /
                (pln.A * a.get_x() + pln.B * a.get_y() + pln.C * a.get_z());
        double x0 = a.get_p().get_x() + a.get_x() * t;
        double y0 = a.get_p().get_y() + a.get_y() * t;
        double z0 = a.get_p().get_z() + a.get_z() * t;
        point3d p0(x0, y0, z0);
        return p0;
    }

//vector3d methods implementations
    vector3d::vector3d(double x, double y, double z, point3d p) : x(x), y(y), z(z),p(p) {}

    double vector3d::length() const {
        return (sqrt(this->x * this->x + this->y * this->y + this->z * this->z));
    }

    double vector3d::scal_prod(const vector3d &v) const {
        return (this->x * v.x + this->y * v.y + this->z * v.z);
    }

    vector3d vector3d::vect_prod(const vector3d &v) const {
        vector3d ret (this->y * v.z - v.y * this->z,
                      this->x * v.z - this->z * v.x,
                      this->x * v.y - v.x * this->y,
                      v.p);
        return ret;
    }

    double vector3d::mixed_prod(const vector3d &v, const vector3d &u) const {
        vector3d b = v.vect_prod(u);
        return this->scal_prod(b);
    }

    bool vector3d::is_collinear(const vector3d &v) const {
        return (std::fabs(this->y * v.z - v.y * this->z) < std::numeric_limits<double>::epsilon() &&
                std::fabs(this->x * v.z - this->z * v.x) < std::numeric_limits<double>::epsilon() &&
                std::fabs(this->x * v.y - v.x * this->y) < std::numeric_limits<double>::epsilon()
        );
    }


//other methods
    plane get_plane(const point3d &p1,const point3d &p2, const point3d &p3) {
        plane pln;
        pln.A = p1.get_y() * (p2.get_z() - p3.get_z()) + p2.get_y() * (p3.get_z() - p1.get_z()) +
                p3.get_y() * (p1.get_z() - p2.get_z());
        pln.B = p1.get_z() * (p2.get_x() - p3.get_x()) + p2.get_z() * (p3.get_x() - p1.get_x()) +
                p3.get_z() * (p1.get_x() - p2.get_x());
        pln.C = p1.get_x() * (p2.get_y() - p3.get_y()) + p2.get_x() * (p3.get_y() - p1.get_y()) +
                p3.get_x() * (p1.get_y() - p2.get_y());
        pln.D = - (p1.get_x() * (p2.get_y() * p3.get_z() - p3.get_y() * p2.get_z()) +
                     p2.get_x() * (p3.get_y() * p1.get_z() - p1.get_y() * p3.get_z()) +
                     p3.get_x() * (p1.get_y() * p2.get_z() - p2.get_y() * p1.get_z())
        );
        return pln;
    }

    bool is_point(std::array<double, 9> &data) {
        point3d p1 {data[0], data[1], data[2]};
        point3d p2 {data[3], data[4], data[5]};
        point3d p3 {data[6], data[7], data[8]};
        return (p1.is_equal(p2) && p1.is_equal(p3));
    }

    bool is_segment(std::array<double, 9> &data) {
        if (is_point(data)) { return false; }
        point3d a(data[0], data[1], data[2]);
        vector3d AB(data[3] - data[0], data[4] - data[1], data[5] - data[2], a);
        vector3d AC(data[6] - data[0], data[7] - data[1], data[8] - data[2], a);
        return AB.is_collinear(AC);
    }

    bool is_triangle(std::array<double, 9> &data) {
        return (!is_point(data) && !is_segment(data));
    }

    point3d to_point(std::array<double, 9> &data) {
        point3d p(data[0], data[1], data[2]);
        return p;
    }

    segment3d to_segment(std::array<double, 9> &data){
        point3d A {data[0], data[1], data[2]};
        point3d B {data[3], data[4], data[5]};
        point3d C {data[6], data[7], data[8]};
        segment3d AB {A, B};
        segment3d AC {A, C};
        segment3d BC {B, C};
        if ((AB.length() > AC.length()) && (AB.length() > BC.length())) { return AB; }
        if ((BC.length() > AC.length()) && (BC.length() > AB.length())) { return BC; }
        return AC;
    }

    triangle3d to_triangle(std::array<double, 9> &data){
        point3d A {data[0], data[1], data[2]};
        point3d B {data[3], data[4], data[5]};
        point3d C {data[6], data[7], data[8]};
        triangle3d t(A, B, C);
        return t;
    }

//intersection function
    bool intersection(std::array<double, 9> &data1, std::array<double, 9> &data2){
        if (is_point(data1)){
            if (is_point(data2)){
                point3d p1 = to_point(data1);
                point3d p2 = to_point(data2);
                return p1.is_equal(p2);
            }
            if (is_segment(data2)){
                point3d p = to_point(data1);
                segment3d s = to_segment(data2);
                return p.is_on_segment(s);
            }
            point3d p = to_point(data1);
            triangle3d t = to_triangle(data2);
            return p.in_triangle(t);
        }

        if (is_point(data2)){
            if (is_segment(data1)){
                point3d p = to_point(data2);
                segment3d s = to_segment(data1);
                return p.is_on_segment(s);
            }
            point3d p = to_point(data2);
            triangle3d t = to_triangle(data1);
            return p.in_triangle(t);
        }

        if (is_segment(data1)){
            if(is_segment(data2)){
                segment3d s1 = to_segment(data1);
                segment3d s2 = to_segment(data2);
                return s1.is_crossing(s2);
            }
            segment3d s = to_segment(data1);
            triangle3d t = to_triangle(data2);
            return s.is_crossing(t);
        }

        if (is_segment(data2)){
            segment3d s = to_segment(data2);
            triangle3d t = to_triangle(data1);
            return s.is_crossing(t);
        }

        triangle3d t1 = to_triangle(data1);
        triangle3d t2 = to_triangle(data2);
        return t1.is_intersect(t2);
    }
}



