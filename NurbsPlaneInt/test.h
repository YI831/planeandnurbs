#pragma once

#include <cmath>
#include <Eigen/Dense>
#include "opennurbs.h"
#include "parasolid_kernel.h"
#include <3rd_party/SISL/include/sisl.h>
#include <3rd_party/SISL/include/sislP.h>


bool StartParasolidSession();
void StopParasolidSession();


using namespace Eigen;
using namespace std;

struct Point {
    double x, y, z;
};
struct Vector3a {
    double x, y, z;

    Vector3a(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vector3a operator-(const Vector3a& v) const {
        return Vector3a(x - v.x, y - v.y, z - v.z);
    }

    Vector3a cross(const Vector3a& v) const {
        return Vector3a(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    double length() const {
        return sqrt(x * x + y * y + z * z);
    }

    Vector3a normalize() const {
        double len = length();
        return Vector3a(x / len, y / len, z / len);
    }
};



struct Matrix4x4 {
    Matrix4d m;

    // 矩阵乘法
    Point multiply(const Point& p) const {
        Point result;
        result.x = m(0, 0) * p.x + m(0, 1) * p.y + m(0, 2) * p.z + m(0, 3);
        result.y = m(1, 0) * p.x + m(1, 1) * p.y + m(1, 2) * p.z + m(1, 3);
        result.z = m(2, 0) * p.x + m(2, 1) * p.y + m(2, 2) * p.z + m(2, 3);
        return result;
    }
    double determinant() const {
        return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) -
            m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
            m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
    }

    // 计算逆矩阵
    Matrix4x4 inverse() const {
        double det = determinant();
        if (det == 0) {
            throw runtime_error("Matrix is singular, cannot compute inverse.");
        }
        // 计算逆矩阵（简化处理，假设矩阵可逆且为标准形式）
        Matrix4x4 inv;
        // 这里使用特定的公式计算逆矩阵的元素
        inv.m(0, 0) = m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1);
        inv.m(0, 1) = m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2);
        inv.m(0, 2) = m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1);
        inv.m(0, 3) = -(inv.m(0, 0) * m(0, 3) + inv.m(0, 1) * m(1, 3) + inv.m(0, 2) * m(2, 3));

        inv.m(1, 0) = m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2);
        inv.m(1, 1) = m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0);
        inv.m(1, 2) = m(0, 2) * m(1, 0) - m(0, 0) * m(1, 2);
        inv.m(1, 3) = -(inv.m(1, 0) * m(0, 3) + inv.m(1, 1) * m(1, 3) + inv.m(1, 2) * m(2, 3));

        inv.m(2, 0) = m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0);
        inv.m(2, 1) = m(0, 1) * m(2, 0) - m(0, 0) * m(2, 1);
        inv.m(2, 2) = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
        inv.m(2, 3) = -(inv.m(2, 0) * m(0, 3) + inv.m(2, 1) * m(1, 3) + inv.m(2, 2) * m(2, 3));

        inv.m(3, 0) = 0; inv.m(3, 1) = 0; inv.m(3, 2) = 0; inv.m(3, 3) = 1; // 齐次坐标的最后一行
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                inv.m(i, j) /= det; // 归一化
            }
        }
        return inv;
    }
};


void CreatNURBS(int p, int q, int n, int m,
    std::vector<Eigen::MatrixXd> Pw,
    std::vector<double> U,
    std::vector<double> V,
    ON_NurbsSurface& nurbs_surf);

Matrix4d computeAffineMatrix(const Vector3d& normal, const Vector3d& point);

Point inverse_affine_transform(const Point& p, const Matrix4d& A);

void PrintCurveControlPoints(const ON_Curve& curve);

int ConvexHull(const ON_Curve& curve);

int ConvexHullOnSurface(const ON_NurbsSurface& nurbS);

bool FindIntersection(ON_Curve& curve, double& tIntersect);

int SplitTheBoundary(ON_Curve* iso_curve, std::vector<double>& para_t);

int SplitTheinnerBoundary(ON_Curve* iso_curve, std::vector<double>& para_t);

ON_3dPoint GetPointOnNURBSSurface(const ON_NurbsSurface& nurbs_surf, double u, double v);

Vector3a GetDUVOnNURBSSurface(const ON_NurbsSurface& nurbs_surf, double u, double v, int d);

PK_ERROR_code_t create_bsurf_from_onnurbs(const ON_NurbsSurface& nurbs, PK_BSURF_t& bsurf);

int ConvexHullOnSplinedSurface(const ON_NurbsSurface& nurbS);

int NormalCone(const ON_NurbsSurface& nurbS);
double objective_function(double u, double v, const ON_NurbsSurface& nurbS);

bool gradient_descent(double& u, double& v, double learning_rate, int max_iterations,
    const ON_NurbsSurface& nurbS, std::vector<double>& tIntersect);
int SplitTheSurface(const ON_NurbsSurface& nurbS, std::vector<std::vector<double>>& para_t);
//void SplitTheSurface(const ON_NurbsSurface& nurbS,std::vector<std::vector<double>>& para_t, int depth = 0);

std::vector<Eigen::MatrixXd>  TransformControlPoints(int n, int m, std::vector<Eigen::MatrixXd> Pw, Matrix4x4 transform);

void FindPointsWithZZero(const ON_NurbsSurface& nurbs_surf, Matrix4x4 transform);

vector<double> traceline(const ON_NurbsSurface& nurbs_surf, double u0, double v0, int dir);
vector<double> tracelineloop(const ON_NurbsSurface& nurbs_surf, double u0, double v0, int dir);

Vector3d SurfaceNormalization(ON_NurbsSurface& surf_nurbs, ON_BoundingBox& bbox, Vector3d point);
Vector3d SurfaceNormalizationt(ON_NurbsSurface& surf_nurbs, ON_BoundingBox& bbox, Vector3d point);
vector<std::vector<Point>> NurbsPlaneInt(double planA, double planB, double planC, float plane_normal[3],ON_NurbsSurface nurbs_surf);
vector<std::vector<Point>> NurbsPlaneOpenCurveInt(double planA, double planB, double planC, float plane_normal[3], ON_NurbsSurface nurbs_surf);
vector<std::vector<Point>> NurbsPlaneLoopInt(double planA, double planB, double planC, float plane_normal[3], ON_NurbsSurface nurbs_surf);
vector<std::vector<Point>> NurbsPlaneInt2(double planA, double planB, double planC, float plane_normal[3], ON_NurbsSurface nurbs_surf, int ModelNumber);
void nurbsintersection_Parasolid(ON_NurbsSurface& nurbs1, ON_NurbsSurface& nurbs2, std::vector<std::vector<double>>& intersection_points);
int Maptraceline(vector<std::vector<Point>> groups);

int whethertraced(vector<std::vector<Point>>viewoutput, Point point_ans);
int whetherlooped(vector<double> p1, double p10, double p11);
int whetherlineontheplane(ON_Curve* iso_curve1, double tol);
void outputlineontheplane(ON_Curve* iso_curve1, vector<std::vector<Point>>& viewopencurve);

double ComputeStepSize(const ON_NurbsSurface& nurbs_surf, double u0, double v0);

void nurbsintersection_OCC(ON_NurbsSurface& nurbs1, ON_NurbsSurface& nurbs2, std::vector<std::vector<double>>& intersection_points);
//int delthepoint(vector<vector<double>> para_uv);

bool writeBMP(const std::string& filename,
    const std::vector<unsigned char>& img,
    int W, int H);
std::string savePointsImage(const std::vector<std::vector<Point>>& groups, int   POINT_RADIUS, int W, int H);

void Boundaryintersection(int curveNumber, ON_Curve* iso_curve1, vector<double>& para_1, double point234[6], vector<std::vector<Point>>& viewoutput, ON_NurbsSurface nurbs_surf, vector<int> values, ON_NurbsSurface nurbs_surf1, std::vector<Point>& Boundary1, double minUknot, double minVknot, double maxUknot, double maxVknot);