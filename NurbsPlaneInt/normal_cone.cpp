#include "normal_cone.h"
#include <algorithm>
#include <cmath>
#include <cassert>
#include <array>

/*noramal cone*/
//! Get u-direction tangent vectors of a BSplineSurface
static void
get_u_control_points(int udegree, Array2D_ONPoint& control_points_u, const Array2D_ONPoint& control_points)
{
    for (int i = 1; i <= control_points_u.N_Rows(); i++)
    {
        for (int j = 1; j <= control_points_u.N_Columns(); j++)
        {
            // std::cout << udegree*(control_points(i+1,j).y-control_points(i,j).y) << std::endl;
            control_points_u(i, j).x = (udegree * (control_points(i + 1, j).x - control_points(i, j).x));
            control_points_u(i, j).y = (udegree * (control_points(i + 1, j).y - control_points(i, j).y));
            control_points_u(i, j).z = (udegree * (control_points(i + 1, j).z - control_points(i, j).z));
        }
    }
}

//! Get v-direction tangent vectors of a BSplineSurface
static void
get_v_control_points(int vdegree, Array2D_ONPoint& control_points_v, const Array2D_ONPoint& control_points)
{
    for (int i = 1; i <= control_points_v.N_Rows(); i++)
    {
        for (int j = 1; j <= control_points_v.N_Columns(); j++)
        {
            control_points_v(i, j).x = (vdegree * (control_points(i, j + 1).x - control_points(i, j).x));
            control_points_v(i, j).y = (vdegree * (control_points(i, j + 1).y - control_points(i, j).y));
            control_points_v(i, j).z = (vdegree * (control_points(i, j + 1).z - control_points(i, j).z));
        }
    }
}

//! Get normal vectors of a BSplineSurface
static void
normalize_control_points(Array2D_ONPoint& control_points, FLOAT default_atol = 1.0e-8)
{
    for (int i = 1; i <= control_points.N_Rows(); i++)
    {
        for (int j = 1; j <= control_points.N_Columns(); j++)
        {
            FLOAT norm = 0.0;
            FLOAT axis[3];
            axis[0] = control_points(i, j).x;
            axis[1] = control_points(i, j).y;
            axis[2] = control_points(i, j).z;
            norm = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
            // if (norm == 0.0)
            if (norm < default_atol)
            {
                // std::cout << "control points at original point!" << std::endl;
            }
            else
            {
                control_points(i, j).x = (axis[0] / norm);
                control_points(i, j).y = (axis[1] / norm);
                control_points(i, j).z = (axis[2] / norm);
            }
        }
    }
}

//! Get a cone using vector sets
static void
generate_cone_simple(FLOAT* axis, FLOAT& theta, Array2D_ONPoint control_points, FLOAT default_atol = 1.0e-8)
{
    axis[0] = 0.0;
    axis[1] = 0.0;
    axis[2] = 0.0;
    for (int i = 1; i <= control_points.N_Rows(); i++)
    {
        for (int j = 1; j <= control_points.N_Columns(); j++)
        {
            axis[0] += control_points(i, j).x;
            axis[1] += control_points(i, j).y;
            axis[2] += control_points(i, j).z;
        }
    }
    axis[0] = axis[0] / control_points.Size();
    axis[1] = axis[1] / control_points.Size();
    axis[2] = axis[2] / control_points.Size();
    FLOAT norm = 0.0;
    norm = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (norm < default_atol)
    {
        theta = M_PI;
        axis[0] = 0.0;
        axis[1] = 0.0;
        axis[2] = 1.0;
        return;
    }
    axis[0] = axis[0] / norm;
    axis[1] = axis[1] / norm;
    axis[2] = axis[2] / norm;

    FLOAT tmp_cos_theta = 1.0;
    for (int i = 1; i <= control_points.N_Rows(); i++)
    {
        for (int j = 1; j <= control_points.N_Columns(); j++)
        {
            FLOAT tmp;
            tmp = control_points(i, j).x * control_points(i, j).x + control_points(i, j).y * control_points(i, j).y + control_points(i, j).z * control_points(i, j).z;
            tmp = sqrt(tmp);
            // if (tmp == 0.0)
            if (tmp < default_atol)
            {
                // std::cerr << "tmp = " << tmp << std::endl;
                continue;
            }
            tmp = (axis[0] * control_points(i, j).x + axis[1] * control_points(i, j).y + axis[2] * control_points(i, j).z) / tmp;
            if (tmp < tmp_cos_theta)
            {
                tmp_cos_theta = tmp;
            }
        }
    }

    theta = acos(tmp_cos_theta);

    // std::cout << "theta simple  " << theta << std::endl;
    // std::cout << axis[0] << "  " << axis[1] << "  " << axis[2] << std::endl;
    // std::cout << control_points.N_Rows() << std::endl;
    // std::cout << control_points.N_Columns() << std::endl;
    // std::cout << control_points.Size() << std::endl;
}


//! Get tangent cone (u direction)
//! axis: the axis of cone
//! theta: the angle of cone
//! control_points: the control points of B-spline surface
void get_surface_u_cone(FLOAT* axis, FLOAT& theta, const Array2D_ONPoint& control_points, FLOAT default_atol)
{
    axis[0] = 0.0;
    axis[1] = 0.0;
    axis[2] = 0.0;
    int u_degree = control_points.N_Rows() - 1;
    int v_degree = control_points.N_Columns() - 1;
    Array2D_ONPoint controlPoints_u(1, u_degree, 1, v_degree + 1);
    get_u_control_points(u_degree, controlPoints_u, control_points);
#if 0
    for (int i = 0; i < controlPoints_u.N_Rows(); i++)
    {
        for (int j = 0; j < controlPoints_u.N_Columns(); j++)
        {
            std::cerr << "tg_cone_u(" << i + 1 << ", " << j + 1 << ") = (" << controlPoints_u(i + 1, j + 1).x << ", "
                << controlPoints_u(i + 1, j + 1).y << ", " << controlPoints_u(i + 1, j + 1).z << ")" << std::endl;
        }
    }
#endif
    normalize_control_points(controlPoints_u, default_atol);
#if 0
    for (int i = 0; i < controlPoints_u.N_Rows(); i++)
    {
        for (int j = 0; j < controlPoints_u.N_Columns(); j++)
        {
            std::cerr << "tg_cone_u(" << i + 1 << ", " << j + 1 << ") = (" << controlPoints_u(i + 1, j + 1).x << ", "
                << controlPoints_u(i + 1, j + 1).y << ", " << controlPoints_u(i + 1, j + 1).z << ")" << std::endl;
        }
    }
#endif
    generate_cone_simple(axis, theta, controlPoints_u, default_atol);
}

//! Get tangent cone (u direction)
//! axis: the axis of cone
//! theta: the angle of cone
//! control_points: the control points of B-spline surface
void get_surface_v_cone(FLOAT* axis, FLOAT& theta, const Array2D_ONPoint& control_points, FLOAT default_atol)
{
    axis[0] = 0.0;
    axis[1] = 0.0;
    axis[2] = 0.0;
    int u_degree = control_points.N_Rows() - 1;
    int v_degree = control_points.N_Columns() - 1;
    Array2D_ONPoint controlPoints_v(1, u_degree + 1, 1, v_degree);
    get_v_control_points(v_degree, controlPoints_v, control_points);
#if 0
    for (int i = 0; i < controlPoints_v.N_Rows(); i++)
    {
        for (int j = 0; j < controlPoints_v.N_Columns(); j++)
        {
            std::cerr << "tg_cone_v(" << i + 1 << ", " << j + 1 << ") = (" << controlPoints_v(i + 1, j + 1).x << ", "
                << controlPoints_v(i + 1, j + 1).y << ", " << controlPoints_v(i + 1, j + 1).z << ")" << std::endl;
        }
    }
#endif
    normalize_control_points(controlPoints_v, default_atol);
#if 0
    for (int i = 0; i < controlPoints_v.N_Rows(); i++)
    {
        for (int j = 0; j < controlPoints_v.N_Columns(); j++)
        {
            std::cerr << "tg_cone_v(" << i + 1 << ", " << j + 1 << ") = (" << controlPoints_v(i + 1, j + 1).x << ", "
                << controlPoints_v(i + 1, j + 1).y << ", " << controlPoints_v(i + 1, j + 1).z << ")" << std::endl;
        }
    }
#endif
    generate_cone_simple(axis, theta, controlPoints_v, default_atol);
}

//! Get normal cone (using the u and v direction cone)
//! axis_u: the axis of u tangent cone
//! axis_v: the axis of v tangent cone
//! axis_norm (return): the axis of normal cone
//! theta_u: the angle of u tangent cone
//! theta_v: the angle of v tangent cone
//! theta_norm (return): the angle of normal cone
int get_norm_cone_uv(FLOAT* axis_u, FLOAT* axis_v, FLOAT* axis_norm, FLOAT theta_u, FLOAT theta_v, FLOAT& theta_norm, FLOAT default_atol)
{
    FLOAT beta;
    beta = axis_u[0] * axis_v[0] + axis_u[1] * axis_v[1] + axis_u[2] * axis_v[2];
    beta = acos(fabs(beta));
    FLOAT tmp = sqrt(sin(theta_u) * sin(theta_u) + 2 * sin(theta_u) * sin(theta_v) * cos(beta) + sin(theta_v) * sin(theta_v));
    tmp = tmp / sin(beta);
    if (fabs(tmp) > 1 || theta_u + theta_v > beta)
    {
        // std::cout << "cannot compute norm cone through uv-cone as u-cone and v-cone are overlapped!" << std::endl;
        theta_norm = M_PI;
        axis_norm[0] = 0.0;
        axis_norm[1] = 0.0;
        axis_norm[2] = 1.0;

        return -1;
    }
    else
    {
        theta_norm = asin(tmp);
    }
    axis_norm[0] = axis_u[1] * axis_v[2] - axis_u[2] * axis_v[1];
    axis_norm[1] = axis_u[2] * axis_v[0] - axis_u[0] * axis_v[2];
    axis_norm[2] = axis_u[0] * axis_v[1] - axis_u[1] * axis_v[0];
    tmp = sqrt(axis_norm[0] * axis_norm[0] + axis_norm[1] * axis_norm[1] + axis_norm[2] * axis_norm[2]);
    if (tmp < default_atol)
    {
        theta_norm = M_PI;
        axis_norm[0] = 0.0;
        axis_norm[1] = 0.0;
        axis_norm[2] = 1.0;

        return -1;
    }
    axis_norm[0] /= tmp;
    axis_norm[1] /= tmp;
    axis_norm[2] /= tmp;
    // std::cout << "tmp  " << tmp << std::endl;
    // std::cout << "theta uv->norm  " << theta_norm << std::endl;
    // std::cout << axis_norm[0] << "  " << axis_norm[1] << "  " << axis_norm[2] << std::endl;
    return 1;
}

//! Get normal cone 
//! axis: the axis of cone
//! theta: the angle of cone
//! control_points: the control points of B-spline surface
void get_surface_uv_norm_cone(FLOAT* axis, FLOAT& theta, const Array2D_ONPoint& control_points, FLOAT default_atol)
{
    axis[0] = 0.0;
    axis[1] = 0.0;
    axis[2] = 0.0;
    FLOAT axis_u[3];
    axis_u[0] = axis_u[1] = axis_u[2] = 0;
    FLOAT theta_u;
    get_surface_u_cone(axis_u, theta_u, control_points, default_atol);
    //std::cout << "u_cone: "<< axis_u[0] << "  " << axis_u[1] << "  " << axis_u[2] <<"; "<< theta_u << std::endl;

    FLOAT axis_v[3];
    axis_v[0] = axis_v[1] = axis_v[2] = 0;
    FLOAT theta_v;
    get_surface_v_cone(axis_v, theta_v, control_points, default_atol);
    //std::cout << "v_cone: " << axis_v[0] << "  " << axis_v[1] << "  " << axis_v[2] << "; " << theta_v << std::endl;

    // int result = get_norm_cone_uv(axis_u, axis_v, axis, theta_u, theta_v, theta, default_atol);
    get_norm_cone_uv(axis_u, axis_v, axis, theta_u, theta_v, theta, default_atol);
}


void transfer_cv(ON_NurbsSurface* surf, Array2D_ONPoint& control_points)
{
    ON_3dPoint cp;
    for (int i = 0; i < surf->CVCount(0); i++)
    {
        for (int j = 0; j < surf->CVCount(1); j++)
        {
            surf->GetCV(i, j, cp);
            control_points(i + 1, j + 1).x = (cp.x);
            control_points(i + 1, j + 1).y = (cp.y);
            control_points(i + 1, j + 1).z = (cp.z);
        }
    }
}

void get_surface_uv_norm_cone_opennurbs(FLOAT* axis, FLOAT& theta, ON_NurbsSurface* surf, FLOAT default_atol)
{
    Array2D_ONPoint control_points(1, surf->CVCount(0), 1, surf->CVCount(1));
    transfer_cv(surf, control_points);

    /*std::cout << "transfer_cv_before: " << "\n";
    ON_3dPoint cp;
    for (int i = 0; i < surf->CVCount(0); i++)
    {
        for (int j = 0; j < surf->CVCount(1); j++)
        {
            surf->GetCV(i, j, cp);
            std::cout << cp.x << ", " << cp.y << ", " << cp.z << "   ";
        }
        std::cout << " \n";
    }std::cout << "\n" << "\n";
    std::cout << "transfer_cv_after: " << "\n";
    for (int i = 0; i < surf->CVCount(0); i++)
    {
        for (int j = 0; j < surf->CVCount(1); j++)
        {
            std::cout << control_points(i + 1, j + 1).x << ", " << control_points(i + 1, j + 1).y << ", " << control_points(i + 1, j + 1).z << "   ";
        }
        std::cout << " \n";
    }std::cout << "\n" << "\n";*/

    get_surface_uv_norm_cone(axis, theta, control_points);
}