#include <vector>
#include <map>
#include <memory>
#include <cstdint>
#include <iostream>

#include "opennurbs.h"

#ifndef FLOAT 
#define FLOAT double
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846

#endif


struct Point_3d {
    double x, y, z;
};

class Array2D_ONPoint
{
public:
    using value_type = Point_3d;

    Array2D_ONPoint() = default;

    Array2D_ONPoint(int iMin, int iMax, int jMin, int jMax)
    {
        Resize(iMin, iMax, jMin, jMax);
    }

    void Resize(int iMin, int iMax, int jMin, int jMax)
    {
         
        iMin_ = iMin; iMax_ = iMax;
        jMin_ = jMin; jMax_ = jMax;

        ni_ = iMax_ - iMin_ + 1;
        nj_ = jMax_ - jMin_ + 1;

        data_.assign(
            static_cast<std::size_t>(ni_) * static_cast<std::size_t>(nj_),
            Point_3d{ 0.0, 0.0, 0.0 }
        );
    }

    Point_3d& operator()(int i, int j)
    {
        return data_[Index(i, j)];
    }

    const Point_3d& operator()(int i, int j) const
    {
        return data_[Index(i, j)];
    }

    // 下标范围
    int IMin() const { return iMin_; }
    int IMax() const { return iMax_; }
    int JMin() const { return jMin_; }
    int JMax() const { return jMax_; }

    int NbI() const { return ni_; }
    int NbJ() const { return nj_; }

    // ---- OCCT 兼容接口（让原代码少改）----
    int N_Rows() const { return ni_; }
    int N_Columns() const { return nj_; }

    std::size_t Size() const
    {
        return data_.size();
    }

    Point_3d* Data() { return data_.data(); }
    const Point_3d* Data() const { return data_.data(); }

private:
    int iMin_ = 1, iMax_ = 0;
    int jMin_ = 1, jMax_ = 0;
    int ni_ = 0, nj_ = 0;

    std::vector<Point_3d> data_;

    std::size_t Index(int i, int j) const
    {
        if (i < iMin_ || i > iMax_ || j < jMin_ || j > jMax_)
            throw std::out_of_range("Array2D_ONPoint index out of range");

        return static_cast<std::size_t>(i - iMin_) * static_cast<std::size_t>(nj_)
            + static_cast<std::size_t>(j - jMin_);
    }
};

/*normal cone*/
//! Get tangent cone (u direction)
//! axis: the axis of cone
//! theta: the angle of cone
//! control_points: the control points of B-spline surface
void get_surface_u_cone(FLOAT* axis, FLOAT& theta, const Array2D_ONPoint& control_points, FLOAT default_atol = 1.0e-8);

//! Get tangent cone (u direction)
//! axis: the axis of cone
//! theta: the angle of cone
//! control_points: the control points of B-spline surface
void get_surface_v_cone(FLOAT* axis, FLOAT& theta, const Array2D_ONPoint& control_points, FLOAT default_atol = 1.0e-8);

//! Get normal cone 
//! axis: the axis of cone
//! theta: the angle of cone
//! control_points: the control points of B-spline surface
void get_surface_uv_norm_cone(FLOAT* axis, FLOAT& theta, const Array2D_ONPoint& control_points, FLOAT default_atol = 1.0e-8);

//! Get normal cone (using the u and v direction cone)
//! axis_u: the axis of u tangent cone
//! axis_v: the axis of v tangent cone
//! axis_norm (return): the axis of normal cone
//! theta_u: the angle of u tangent cone
//! theta_v: the angle of v tangent cone
//! theta_norm (return): the angle of normal cone
int get_norm_cone_uv(FLOAT* axis_u, FLOAT* axis_v, FLOAT* axis_norm, FLOAT theta_u, FLOAT theta_v, FLOAT& theta_norm, FLOAT default_atol = 1.0e-8);

//!transfer control pints in ON_NurbsSurface* surf to Array2D_ONPoint& control_points
void transfer_cv(ON_NurbsSurface* surf, Array2D_ONPoint& control_points);

//! Get normal cone 
//! axis: the axis of cone
//! theta: the angle of cone
//! control_points: the control points of B-spline surface
void get_surface_uv_norm_cone_opennurbs(FLOAT * axis, FLOAT & theta, ON_NurbsSurface * surf, FLOAT default_atol = 1.0e-8);  
