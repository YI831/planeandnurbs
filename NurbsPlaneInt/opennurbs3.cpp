#include "test.h"
#include "normal_cone.h"
#include "parasolid_kernel.h"

#include <iostream>
#include <fstream>


void CreatNURBS(int p, int q, int n, int m,
    std::vector<Eigen::MatrixXd> Pw,
    std::vector<double> U,
    std::vector<double> V,
    ON_NurbsSurface& nurbs_surf)
{

    /*Bsurf:U,V,ConPnts*/
    // 设置曲面的度数
    int degree_u = p; // U 方向的度数
    int degree_v = q; // V 方向的度数
    int order_u = degree_u + 1;
    int order_v = degree_v + 1;
    // 设置曲面的结构类型
    int cv_count_u = n + 1; // U 方向的控制点数量
    int cv_count_v = m + 1; // V 方向的控制点数量
    int dim = 3; // 控制点的维数（在三维空间中每个控制点都有三个坐标）

    // 创建一个双二次 NURBS 曲面对象
    //surf.Create(3, // 控制点维度（3 表示三维控制点）
    //    FALSE, // 是否使用齐次坐标
    //    cv_count_u, // U 方向上的控制点数量
    //    cv_count_v, // V 方向上的控制点数量
    //    cv_count_u * cv_count_v, // 总的控制点数量
    //    FALSE // 是否创建闭合曲面
    //);
    //ON_NurbsSurface nurbs_surf(dim, 0, order_u, order_v, cv_count_u, cv_count_v);
    nurbs_surf.Create(dim, 0, order_u, order_v, cv_count_u, cv_count_v);

    // 定义控制点数组
    ON_3dPointArray control_points;
    for (int i = 0; i < cv_count_u; i++) {
        for (int j = 0; j < cv_count_v; j++) {
            control_points.Append(ON_3dPoint(Pw[0](i, j), Pw[1](i, j), Pw[2](i, j)));
        }
    }

    // 将控制点添加到曲面对象中
    for (int i = 0; i < cv_count_u; i++) {
        for (int j = 0; j < cv_count_v; j++) {
            nurbs_surf.SetCV(i, j, control_points[i * cv_count_v + j]);
        }
    }

    /*std::cout << "KnotU:" << std::endl;
    for (int i = 0; i < cv_count_u + order_u; i++)
        std::cout << Bsurf.U[i] << ", ";
    std::cout << "\n ";
    std::cout << "KnotV:" << std::endl;
    for (int i = 0; i < cv_count_v + order_v; i++)
        std::cout << Bsurf.V[i] << ", ";
    std::cout << "\n ";*/

    // 设置 U 方向上的节点
    //double u_knots[] = { 0., 0, 0.25, 0.75, 1.0, 1.0 }; // U 方向上的节点
    for (int i = 0; i < cv_count_u + order_u - 2; i++)
        nurbs_surf.SetKnot(0, i, U[i + 1]);


    // 设置 V 方向上的节点
    //double v_knots[] = { 0., 0., 0.2, 0.4, 0.6,0.8,1.0, 1.0}; // V 方向上的节点
    for (int i = 0; i < cv_count_v + order_v - 2; i++)
        nurbs_surf.SetKnot(1, i, V[i + 1]);

}

//class NURBSSurface {
//public:
//    std::vector<Eigen::MatrixXd> Pw;  // 控制点
//                                      // 权重
//    std::vector<double> knotVectorU;             // U方向的节点向量
//    std::vector<double> knotVectorV;             // V方向的节点向量
//    int degreeU;                                 // U方向的维度
//    int degreeV;                                 // V方向的维度
//    int numControlPointsU;                       // U方向的控制点数量
//    int numControlPointsV;                       // V方向的控制点数量
//
//    // 构造函数
//    NURBSSurface(
//        std::vector<Eigen::MatrixXd> cps,
//        std::vector<double> ku,
//        std::vector<double> kv,
//        int du, int dv, int nU, int nV)
//        : Pw(cps),  knotVectorU(ku), knotVectorV(kv),
//        degreeU(du), degreeV(dv), numControlPointsU(nU), numControlPointsV(nV) {}
//
//    // 计算基函数值 (递归定义的B样条基函数)
//    double basisFunction(int i, int p, double t, const std::vector<double>& knotVector) {
//        if (p == 0) {
//            return (knotVector[i] <= t && t < knotVector[i + 1]) ? 1.0 : 0.0;
//        }
//        else {
//            double leftTerm = (t - knotVector[i]) / (knotVector[i + p] - knotVector[i]) * basisFunction(i, p - 1, t, knotVector);
//            double rightTerm = (knotVector[i + p + 1] - t) / (knotVector[i + p + 1] - knotVector[i + 1]) * basisFunction(i + 1, p - 1, t, knotVector);
//            return leftTerm + rightTerm;
//        }
//    }
//
//    // 计算NURBS曲面上的点
//    Eigen::Vector3d evaluate(double u, double v) {
//        Eigen::Vector3d numerator(0, 0, 0); // 用于累加分子部分
//        double denominator = 0.0;           // 分母部分
//
//        // 双重循环遍历所有控制点
//        for (int i = 0; i < numControlPointsU; ++i) {
//            for (int j = 0; j < numControlPointsV; ++j) {
//                // 计算U和V方向上的基函数值
//                double Ni = basisFunction(i, degreeU, u, knotVectorU);
//                double Nj = basisFunction(j, degreeV, v, knotVectorV);
//
//                // 计算权重
//                double w_ij = 1;
//                double B = Ni * Nj * w_ij;
//
//                // 累加控制点影响到分子和分母
//                numerator += B * Pw[i * numControlPointsV + j];
//                denominator += B;
//            }
//        }
//
//        // 最终计算出曲面上的点（加权平均）
//        return numerator / denominator;
//    }
//};



// 仿射变换函数
// 计算平面的仿射变换矩阵，将平面转换到XOY平面
Matrix4d computeAffineMatrix(const Vector3d& normal, const Vector3d& point) {
    // 将法向量归一化
    Vector3d n = normal.normalized();

    // 找到旋转矩阵
    Vector3d zAxis(0, 0, 1); // XOY平面的法向量
    Vector3d axis = n.cross(zAxis); // 旋转轴为两个法向量的叉积
    double angle = acos(n.dot(zAxis)); // 旋转角为两个法向量的夹角
    Matrix3d rotation = Matrix3d::Identity();

    if (axis.norm() > 1e-6) {
        axis.normalize();
        // 旋转矩阵使用 Rodrigues 公式
        Matrix3d K;
        K << 0, -axis.z(), axis.y(),
            axis.z(), 0, -axis.x(),
            -axis.y(), axis.x(), 0;
        rotation = Matrix3d::Identity() + sin(angle) * K + (1 - cos(angle)) * K * K;
    }

    // 创建4x4旋转矩阵
    Matrix4d affineMatrix = Matrix4d::Identity();
    affineMatrix.block<3, 3>(0, 0) = rotation;

    // 计算平移矩阵
    Vector3d translation = -rotation * point; // 将平面上的点移动到原点
    affineMatrix.block<3, 1>(0, 3) = translation;
    // std::cout << "The affineMatrix." << affineMatrix << std::endl;
    return affineMatrix;

}




Vector3d  SurfaceNormalization(ON_NurbsSurface& surf_nurbs, ON_BoundingBox& bbox, Vector3d point)
{
    /*Normalize Knots*/
    int knotsize1 = surf_nurbs.KnotCount(0);

    double u1min = surf_nurbs.Knot(0, 0);
    double u1max = surf_nurbs.Knot(0, knotsize1 - 1);

    double rangeu1 = u1max - u1min;
    for (int i = 0; i < knotsize1; i++)
    {
        double pastval = surf_nurbs.Knot(0, i) - u1min;
        surf_nurbs.SetKnot(0, i, pastval / rangeu1);
    }

    int knotsize2 = surf_nurbs.KnotCount(1);
    double v1min = surf_nurbs.Knot(1, 0);
    double v1max = surf_nurbs.Knot(1, knotsize2 - 1);

    double rangev1 = v1max - v1min;
    for (int i = 0; i < knotsize2; i++)
    {
        double pastval = surf_nurbs.Knot(1, i) - v1min;
        surf_nurbs.SetKnot(1, i, pastval / rangev1);
    }


    /*Normalize control point*/
    ON_3dPointArray control_points;
    for (int i = 0; i < surf_nurbs.CVCount(0); i++)
    {
        for (int j = 0; j < surf_nurbs.CVCount(1); j++)
        {
            ON_3dPoint control_point;
            surf_nurbs.GetCV(i, j, control_point);
            control_points.Append(control_point);
        }
    }

    // 获取控制点数组的范围
    bbox.Set(control_points);

    double xmin = bbox.m_min.x;
    double ymin = bbox.m_min.y;
    double zmin = bbox.m_min.z;
    double xmax = bbox.m_max.x;
    double ymax = bbox.m_max.y;
    double zmax = bbox.m_max.z;

    double max_size = (std::max)(xmax - xmin, ymax - ymin);
    max_size = (std::max)(max_size, zmax - zmin);

    /*double cube_size = 1.0;
    double transX = (point[0] - xmin) / ((xmax - xmin) * cube_size);
    double transY = (point[1] - ymin) / ((ymax - ymin) * cube_size);
    double transZ = (point[2] - zmin) / ((zmax - zmin) * cube_size);*/
    double cube_size = 1.0;
    double transX = (point[0] - xmin) / max_size * cube_size;
    double transY = (point[1] - ymin) / max_size * cube_size;
    double transZ = (point[2] - zmin) / max_size * cube_size;

    // 归一化控制点
    for (int i = 0; i < surf_nurbs.CVCount(0); i++)
    {
        for (int j = 0; j < surf_nurbs.CVCount(1); j++)
        {
            ON_3dPoint cp;
            surf_nurbs.GetCV(i, j, cp);
            /*  cp.x = (cp.x - xmin) / (xmax - xmin) * cube_size-transX;
              cp.y = (cp.y - ymin) / (ymax - ymin) * cube_size-transY;
              cp.z = (cp.z - zmin) / (zmax - zmin) * cube_size-transZ;*/
            cp.x = (cp.x - xmin) / max_size * cube_size - transX;
            cp.y = (cp.y - ymin) / max_size * cube_size - transY;
            cp.z = (cp.z - zmin) / max_size * cube_size - transZ;
            surf_nurbs.SetCV(i, j, cp);
        }
    }
    Vector3d pointTrans(transX, transY, transZ);
    return pointTrans;

}

Vector3d  SurfaceNormalizationt(ON_NurbsSurface& surf_nurbs, ON_BoundingBox& bbox, Vector3d point)
{
    /*Normalize Knots*/
    int knotsize1 = surf_nurbs.KnotCount(0);

    double u1min = surf_nurbs.Knot(0, 0);
    double u1max = surf_nurbs.Knot(0, knotsize1 - 1);

    double rangeu1 = u1max - u1min;
    for (int i = 0; i < knotsize1; i++)
    {
        double pastval = surf_nurbs.Knot(0, i) - u1min;
        surf_nurbs.SetKnot(0, i, pastval / rangeu1);
    }

    int knotsize2 = surf_nurbs.KnotCount(1);
    double v1min = surf_nurbs.Knot(1, 0);
    double v1max = surf_nurbs.Knot(1, knotsize2 - 1);

    double rangev1 = v1max - v1min;
    for (int i = 0; i < knotsize2; i++)
    {
        double pastval = surf_nurbs.Knot(1, i) - v1min;
        surf_nurbs.SetKnot(1, i, pastval / rangev1);
    }


    /*Normalize control point*/
    ON_3dPointArray control_points;
    for (int i = 0; i < surf_nurbs.CVCount(0); i++)
    {
        for (int j = 0; j < surf_nurbs.CVCount(1); j++)
        {
            ON_3dPoint control_point;
            surf_nurbs.GetCV(i, j, control_point);
            control_points.Append(control_point);
        }
    }

    // 获取控制点数组的范围
    bbox.Set(control_points);

    double xmin = bbox.m_min.x;
    double ymin = bbox.m_min.y;
    double zmin = bbox.m_min.z;
    double xmax = bbox.m_max.x;
    double ymax = bbox.m_max.y;
    double zmax = bbox.m_max.z;

    double max_size = (std::max)(xmax - xmin, ymax - ymin);
    max_size = (std::max)(max_size, zmax - zmin);

    /*double cube_size = 1.0;
    double transX = (point[0] - xmin) / ((xmax - xmin) * cube_size);
    double transY = (point[1] - ymin) / ((ymax - ymin) * cube_size);
    double transZ = (point[2] - zmin) / ((zmax - zmin) * cube_size);*/
    double cube_size = 1.0;
    double transX = (0 - xmin) / max_size * cube_size;
    double transY = (0 - ymin) / max_size * cube_size;
    double transZ = (0 - zmin) / max_size * cube_size;

    // 归一化控制点
    for (int i = 0; i < surf_nurbs.CVCount(0); i++)
    {
        for (int j = 0; j < surf_nurbs.CVCount(1); j++)
        {
            ON_3dPoint cp;
            surf_nurbs.GetCV(i, j, cp);
            /*  cp.x = (cp.x - xmin) / (xmax - xmin) * cube_size-transX;
              cp.y = (cp.y - ymin) / (ymax - ymin) * cube_size-transY;
              cp.z = (cp.z - zmin) / (zmax - zmin) * cube_size-transZ;*/
            cp.x = (cp.x - xmin) / max_size * cube_size - transX;
            cp.y = (cp.y - ymin) / max_size * cube_size - transY;
            cp.z = (cp.z - zmin) / max_size * cube_size - transZ;
            surf_nurbs.SetCV(i, j, cp);
        }
    }
    Vector3d pointTrans(transX, transY, transZ);
    return pointTrans;

}

// 执行逆变换
Point inverse_affine_transform(const Point& p, const Matrix4d& A) {

    Matrix4d Ainv = A.inverse();

    // 减去平移向量
    Vector3d translation = A.block<3, 1>(0, 3);
    // Point b = { translation[0], translation[1], translation[2] };
    // Point adjusted_point = { p.x - b.x, p.y - b.y, p.z - b.z };

    // Eigen::Vector4d point_4d(adjusted_point.x, adjusted_point.y, adjusted_point.z, 1);
    Eigen::Vector4d point_4d(p.x, p.y, p.z, 1);
    Eigen::Vector4d result = Ainv * point_4d;
    // 应用逆变换
    Point output_point;
    output_point.x = result(0);  // 提取 x
    output_point.y = result(1);  // 提取 y
    output_point.z = result(2);  // 提取 z
    return output_point;
}

void PrintCurveControlPoints(const ON_Curve& curve) {
    const ON_NurbsCurve* nurbs_curve = ON_NurbsCurve::Cast(&curve);
    if (nurbs_curve) {
        int control_point_count = nurbs_curve->CVCount();
        std::cout << "NURBS Curve Control Points (" << control_point_count << " points):" << std::endl;

        for (int i = 0; i < control_point_count; ++i) {
            ON_3dPoint control_point;
            nurbs_curve->GetCV(i, control_point);  // 获取控制点
            std::cout << "Control Point " << i << ": ("
                << control_point.x << ", "
                << control_point.y << ", "
                << control_point.z << ")" << std::endl;
        }
    }
    else {
        std::cout << "The curve is not a NURBS curve." << std::endl;
    }
}

int ConvexHull(const ON_Curve& curve) {
    const ON_NurbsCurve* nurbs_curve = ON_NurbsCurve::Cast(&curve);
    ON_3dPoint control_point;
    ON_3dPoint control_point1;
    nurbs_curve->GetCV(0, control_point);
    double zmin = control_point.z;
    double zmax = control_point.z;
    nurbs_curve->GetCV(1, control_point);
    if (nurbs_curve) {
        int control_point_count = nurbs_curve->CVCount();
        for (int i = 0; i < control_point_count; ++i) {

            nurbs_curve->GetCV(i, control_point);  // 获取控制点
            if (control_point.z <= zmin) {
                zmin = control_point.z;
            }
            if (control_point.z >= zmax) {
                zmax = control_point.z;
            }
        }
        /* std::cout << "The zmin=" <<zmin<< std::endl;
         std::cout << "The zmax=" <<zmax<< std::endl;*/
        if ((zmin * zmax) <= 1e-9) {
            // std::cout << "The ConvexHull crosses z=zero." << std::endl;

            return 1;
        }
        else {
            return 0;
        }

    }
    else {
        // std::cout << "The curve does not cross z=zero." << std::endl;
        return 0;
    }
}

int ConvexHullOnSurface(const ON_NurbsSurface& nurbS) {
    const ON_NurbsSurface* nurbSurface = ON_NurbsSurface::Cast(&nurbS);
    int u_count = nurbSurface->CVCount(0);  // 获取 U 方向的控制点数
    int v_count = nurbSurface->CVCount(1);  // 获取 V 方向的控制点数
    ON_3dPoint cv1;
    ON_3dPoint cv2;
    double dv;
    nurbSurface->GetCV(0, 0, cv1);
    double zmin=cv1.z;
    double zmax=cv1.z;
    int turnu = 0;
    int turnv = 0;
    // 遍历所有控制点
    for (int i = 0; i < u_count; ++i)
    {
        int turn1 = 0;
        int turn2 = 0;
        nurbSurface->GetCV(i, 0, cv1);
        cv2 = cv1;
        nurbSurface->GetCV(i, 1, cv1);
        dv = cv1.z - cv2.z;
        if (cv2.z > zmax) {
            zmax = cv2.z;
        }
        if (cv2.z < zmin) {
            zmin = cv2.z;
        }
        if (dv > 0) {
            turn1 = 1;
        }
        else if (dv < 0) {
            turn2 = -1;
        }
        for (int j = 2; j < v_count; ++j)
        {
            cv2 = cv1;
            nurbSurface->GetCV(i, j, cv1);
            dv = cv1.z - cv2.z;
            if (cv2.z > zmax) {
                zmax = cv2.z;
            }
            if (cv2.z < zmin) {
                zmin = cv2.z;
            }
            if (dv > 0) {
                turn1 = turn1 + 1;
            }
            else if (dv < 0) {
                turn2 = turn2 - 1;
            }
        }
        if (turn1 * turn2 < 0) {
            turnv = turnv + 1;
        }
    }
    if (zmin * zmax > 0) {
        return 0;
    }

    for (int i = 0; i < v_count; ++i)
    {
        int turn1 = 0;
        int turn2 = 0;
        nurbSurface->GetCV(0, i, cv1);
        cv2 = cv1;
        nurbSurface->GetCV(1, i, cv1);
        dv = cv1.z - cv2.z;
        if (dv > 0) {
            turn1 = 1;
        }
        else if (dv < 0) {
            turn2 = -1;
        }
        for (int j = 2; j < u_count; ++j)
        {
            cv2 = cv1;
            nurbSurface->GetCV(j, i, cv1);
            dv = cv1.z - cv2.z;
            if (dv > 0) {
                turn1 = turn1 + 1;
            }
            else if (dv < 0) {
                turn2 = turn2 - 1;
            }
        }
        if (turn1 * turn2 < 0) {
            turnu = turnu - 1;
        }
    }
    if (turnu * turnv < 0) {
        return 1;
    }
    return 0;
}


// 使用牛顿法求 NURBS 曲线与平面的交点
bool FindIntersection(ON_Curve& curve, double& tIntersect) {
    double t = (curve.Domain().Min() + curve.Domain().Max()) / 2.0;  // 初始猜测
    const double epsilon = 1e-8;  // 收敛精度
    const int maxIterations = 1000;

    for (int i = 0; i < maxIterations; ++i) {
        double point321[6];
        bool success = curve.Evaluate(t, 1, 3, point321);

        // 计算 F(t)
        double F = point321[2];  // 我们希望 z(t) = 0
        if (abs(F) < epsilon) {
            tIntersect = t;
            return true;  // 找到交点
        }

        // 计算导数 F'(t)
        double dz;
        dz = point321[5];

        // 防止除以零
        if (std::fabs(dz) < epsilon) {
            //std::cout << "Warning: Derivative is too small, might not converge." << std::endl;
            return false;  // 导数过小，可能不收敛
        }

        // 更新 t
        double tNext = t - F / dz;

        curve.Evaluate(tNext, 1, 3, point321);
        double Ft = point321[2];

        // 检查收敛
        if (abs(Ft) < epsilon) {
            tIntersect = tNext;
            return true;  // 找到交点
        }

        t = tNext;  // 更新 t
    }

    return false;  // 未找到交点
}


int Coplanarity(const ON_Curve& curve) {
    const ON_NurbsCurve* nurbs_curve = ON_NurbsCurve::Cast(&curve);
    ON_3dPoint control_point;
    ON_3dPoint control_point1;
    nurbs_curve->GetCV(0, control_point);
    double zmin = control_point.z;
    double zmax = control_point.z;
    nurbs_curve->GetCV(1, control_point);
    if (nurbs_curve) {
        int control_point_count = nurbs_curve->CVCount();
        for (int i = 0; i < control_point_count; ++i) {
 
            nurbs_curve->GetCV(i, control_point);  // 获取控制点
            if (control_point.z <= zmin) {
                zmin = control_point.z;
            }
            if (control_point.z >= zmax) {
                zmax = control_point.z;
            }
        }
        /* std::cout << "The zmin=" <<zmin<< std::endl;
         std::cout << "The zmax=" <<zmax<< std::endl;*/
        if (zmin < 1e-6 && zmax <1e-6 && zmin > -1e-6 && zmax > -1e-6) {
            // std::cout << "The ConvexHull crosses z=zero." << std::endl;

            return 1;
        }
        else {
            return 0;
        }

    }
    else {
        // std::cout << "The curve does not cross z=zero." << std::endl;
        return 0;
    }
}




int SplitTheBoundary(ON_Curve* iso_curve, std::vector<double>& para_t) {
    double tol = 1e-4 ;
    ON::Begin(); // 初始化OpenNURBS库
    if (ConvexHull(*iso_curve) == 0) {
        return 0;
    }
    /*if (Coplanarity(*iso_curve) == 1) {
        para_t.push_back(iso_curve->Domain().Max());
        para_t.push_back(iso_curve->Domain().Min());
        return 0;
    }*/
    if (Coplanarity(*iso_curve) == 1) {
        para_t.push_back((iso_curve->Domain().Max() + iso_curve->Domain().Min()) / 2.0);
        return 0;
    }
    if (iso_curve) {
        // std::cout << "IsoCurve extracted successfully." << std::endl;

         // 提取等参线后，进行曲线分割
        double split_param = 0.5 * (iso_curve->Domain().Min() + iso_curve->Domain().Max());  // 在曲线的中点处分割
        ON_Curve* left_curve = nullptr;
        ON_Curve* right_curve = nullptr;
        iso_curve->Split(split_param, left_curve, right_curve);
        double tIntersect;
        double point321[6];

        /*   if (iso_curve->Split(split_param, left_curve, right_curve)) {
               std::cout << "Curve split successfully!" << std::endl;
               std::cout << "Left curve domain: " << left_curve->Domain().Min() << " to " << left_curve->Domain().Max() << std::endl;
               std::cout << "Right curve domain: " << right_curve->Domain().Min() << " to " << right_curve->Domain().Max() << std::endl;
           }
           else {
               std::cerr << "Failed to split the curve!" << std::endl;
           }

           std::cout << "ConvexHull: " << ConvexHull(*iso_curve) << "\n";
           std::cout << "ConvexHull___: " << iso_curve->Domain().Max() - iso_curve->Domain().Min() << "\n";*/
        if (ConvexHull(*iso_curve) == 1 && (iso_curve->Domain().Max() - iso_curve->Domain().Min()) < tol) {
            double tIntersect = (iso_curve->Domain().Max() + iso_curve->Domain().Min()) / 2.0;
            //std::cout << "tIntersect_0: " << tIntersect << "\n";
            bool test = true;
            test = FindIntersection(*iso_curve, tIntersect);
            //std::cout << "tIntersect_1: " << tIntersect << "\n";
            if (test == true) {
                bool samepara = true;
                for (int i = 0; i < para_t.size(); i++) {
                    if (abs(tIntersect - para_t[i]) < 1e-4) {
                        samepara = false;
                    }
                }
                if (samepara == true) {
                    para_t.push_back(tIntersect);
                }
            }
            // PrintCurveControlPoints(*iso_curve);
             //std::cout << "Point with z = 0 found at (t = " << tIntersect << "): ";
            double point321[6];
            bool success = iso_curve->Evaluate(tIntersect, 1, 3, point321);
            // 输出曲线在参数 t 处的点
            /*std::cout << "Point at t = " << tIntersect << ": ("
                << point321[0] << ", " << point321[1] << ", " << point321[2] << ")" << std::endl;
            return;*/
            return 1;
        }
        /* PrintCurveControlPoints(*iso_curve);
         PrintCurveControlPoints(*left_curve);
         PrintCurveControlPoints(*right_curve);*/
        if (ConvexHull(*iso_curve) == 1 && (iso_curve->Domain().Max() - iso_curve->Domain().Min()) >= tol) {
            // PrintCurveControlPoints(*iso_curve);
            if (ConvexHull(*left_curve) == 1 && (left_curve->Domain().Max() - left_curve->Domain().Min()) >= tol / 2.0) {
                /*if (Coplanarity(*left_curve) == 1) {
                    para_t.push_back(left_curve->Domain().Max());
                    para_t.push_back(left_curve->Domain().Min());
                    return 0;
                }*/
                //FindIntersection(*left_curve, tIntersect);
                //PrintCurveControlPoints(*left_curve);
                //std::cout << "Point with z = 0 found at (t = " << tIntersect << "): ";
                SplitTheBoundary(left_curve, para_t);
                // bool success = left_curve->Evaluate(tIntersect, 1, 3, point321);
                 // 输出曲线在参数 t 处的点
                // std::cout << "Point at t = " << tIntersect << ": ("
                  //   << point321[0] << ", " << point321[1] << ", " << point321[2] << ")" << std::endl;

            };
            if (ConvexHull(*right_curve) == 1 && (right_curve->Domain().Max() - right_curve->Domain().Min()) >= tol / 2.0) {
                /*if (Coplanarity(*right_curve) == 1) {
                    para_t.push_back(right_curve->Domain().Max());
                    para_t.push_back(right_curve->Domain().Min());
                    return 0;
                }*/
                //    FindIntersection(*right_curve, tIntersect);
                //    //PrintCurveControlPoints(*right_curve);
                //    std::cout << "Point with z = 0 found at (t = " << tIntersect << "): ";
                SplitTheBoundary(right_curve, para_t);
                //    bool success = right_curve->Evaluate(tIntersect, 1, 3, point321);
                //    // 输出曲线在参数 t 处的点
                //    std::cout << "Point at t = " << tIntersect << ": ("
                //        << point321[0] << ", " << point321[1] << ", " << point321[2] << ")" << std::endl;
            }

        }
        //// 清理内存
        if (left_curve != nullptr) {
            delete left_curve;
            left_curve = nullptr;  // 清空指针以防止悬挂指针
        }

        if (right_curve != nullptr) {
            delete right_curve;
            right_curve = nullptr;  // 同样清空指针
        }

    }
    else {
        std::cerr << "Failed to extract IsoCurve!" << std::endl;
    }
    ON::End();
}

int SplitTheinnerBoundary(ON_Curve* iso_curve, std::vector<double>& para_t) {
    double tol = 1e-3;
    ON::Begin(); // 初始化OpenNURBS库
    if (ConvexHull(*iso_curve) == 0) {
        return 0;
    }
    /*if (Coplanarity(*iso_curve) == 1) {
        para_t.push_back(iso_curve->Domain().Max());
        para_t.push_back(iso_curve->Domain().Min());
        return 0;
    }*/
    if (Coplanarity(*iso_curve) == 1) {
        para_t.push_back((iso_curve->Domain().Max() + iso_curve->Domain().Min()) / 2.0);
        return 0;
    }
    if (iso_curve) {
        // std::cout << "IsoCurve extracted successfully." << std::endl;

         // 提取等参线后，进行曲线分割
        double split_param = 0.5 * (iso_curve->Domain().Min() + iso_curve->Domain().Max());  // 在曲线的中点处分割
        ON_Curve* left_curve = nullptr;
        ON_Curve* right_curve = nullptr;
        iso_curve->Split(split_param, left_curve, right_curve);
        double tIntersect;
        double point321[6];

        /*   if (iso_curve->Split(split_param, left_curve, right_curve)) {
               std::cout << "Curve split successfully!" << std::endl;
               std::cout << "Left curve domain: " << left_curve->Domain().Min() << " to " << left_curve->Domain().Max() << std::endl;
               std::cout << "Right curve domain: " << right_curve->Domain().Min() << " to " << right_curve->Domain().Max() << std::endl;
           }
           else {
               std::cerr << "Failed to split the curve!" << std::endl;
           }

           std::cout << "ConvexHull: " << ConvexHull(*iso_curve) << "\n";
           std::cout << "ConvexHull___: " << iso_curve->Domain().Max() - iso_curve->Domain().Min() << "\n";*/
        if (ConvexHull(*iso_curve) == 1 && (iso_curve->Domain().Max() - iso_curve->Domain().Min()) < tol) {
            double tIntersect = (iso_curve->Domain().Max() + iso_curve->Domain().Min()) / 2.0;
            //std::cout << "tIntersect_0: " << tIntersect << "\n";
            bool test = true;
            test = FindIntersection(*iso_curve, tIntersect);
            //std::cout << "tIntersect_1: " << tIntersect << "\n";
            if (test == true) {
                bool samepara = true;
                for (int i = 0; i < para_t.size(); i++) {
                    if (abs(tIntersect - para_t[i]) < 1e-4) {
                        samepara = false;
                    }
                }
                if (samepara == true) {
                    para_t.push_back(tIntersect);
                }
            }
            // PrintCurveControlPoints(*iso_curve);
             //std::cout << "Point with z = 0 found at (t = " << tIntersect << "): ";
            double point321[6];
            bool success = iso_curve->Evaluate(tIntersect, 1, 3, point321);
            // 输出曲线在参数 t 处的点
            /*std::cout << "Point at t = " << tIntersect << ": ("
                << point321[0] << ", " << point321[1] << ", " << point321[2] << ")" << std::endl;
            return;*/
            return 1;
        }
        /* PrintCurveControlPoints(*iso_curve);
         PrintCurveControlPoints(*left_curve);
         PrintCurveControlPoints(*right_curve);*/
        if (ConvexHull(*iso_curve) == 1 && (iso_curve->Domain().Max() - iso_curve->Domain().Min()) >= tol) {
            // PrintCurveControlPoints(*iso_curve);
            if (ConvexHull(*left_curve) == 1 && (left_curve->Domain().Max() - left_curve->Domain().Min()) >= tol / 2.0) {
                /*if (Coplanarity(*left_curve) == 1) {
                    para_t.push_back(left_curve->Domain().Max());
                    para_t.push_back(left_curve->Domain().Min());
                    return 0;
                }*/
                //FindIntersection(*left_curve, tIntersect);
                //PrintCurveControlPoints(*left_curve);
                //std::cout << "Point with z = 0 found at (t = " << tIntersect << "): ";
                SplitTheinnerBoundary(left_curve, para_t);
                // bool success = left_curve->Evaluate(tIntersect, 1, 3, point321);
                 // 输出曲线在参数 t 处的点
                // std::cout << "Point at t = " << tIntersect << ": ("
                  //   << point321[0] << ", " << point321[1] << ", " << point321[2] << ")" << std::endl;

            };
            if (ConvexHull(*right_curve) == 1 && (right_curve->Domain().Max() - right_curve->Domain().Min()) >= tol / 2.0) {
                /*if (Coplanarity(*right_curve) == 1) {
                    para_t.push_back(right_curve->Domain().Max());
                    para_t.push_back(right_curve->Domain().Min());
                    return 0;
                }*/
                //    FindIntersection(*right_curve, tIntersect);
                //    //PrintCurveControlPoints(*right_curve);
                //    std::cout << "Point with z = 0 found at (t = " << tIntersect << "): ";
                SplitTheinnerBoundary(right_curve, para_t);
                //    bool success = right_curve->Evaluate(tIntersect, 1, 3, point321);
                //    // 输出曲线在参数 t 处的点
                //    std::cout << "Point at t = " << tIntersect << ": ("
                //        << point321[0] << ", " << point321[1] << ", " << point321[2] << ")" << std::endl;
            }

        }
        //// 清理内存
        if (left_curve != nullptr) {
            delete left_curve;
            left_curve = nullptr;  // 清空指针以防止悬挂指针
        }

        if (right_curve != nullptr) {
            delete right_curve;
            right_curve = nullptr;  // 同样清空指针
        }

    }
    else {
        std::cerr << "Failed to extract IsoCurve!" << std::endl;
    }
    ON::End();
}


ON_3dPoint GetPointOnNURBSSurface(const ON_NurbsSurface& nurbs_surf, double u, double v) {
    return nurbs_surf.PointAt(u, v); // 计算 (u, v) 参数对应的点
}

Vector3a GetDUVOnNURBSSurface(const ON_NurbsSurface& nurbs_surf, double u, double v, int d) {

    ON_3dPoint point;
    Vector3a du, dv, duu, duv, dvv;
    //计算曲面上 (u, v) 位置的点以及 u 和 v 方向的导数
   // 这里只要求一次导数，因此传递 der_count=1 和 return_der=1
    double val[18];
    bool success = nurbs_surf.Evaluate(u, v, 1, 3, val);
    du.x = val[3];
    du.y = val[4];
    du.z = val[5];

    dv.x = val[6];
    dv.y = val[7];
    dv.z = val[8];

    duu.x = val[9];
    duu.y = val[10];
    duu.z = val[11];

    duv.x = val[12];
    duv.y = val[13];
    duv.z = val[14];

    dvv.x = val[15];
    dvv.y = val[16];
    dvv.z = val[17];
    if (success) {
        // 输出结果
        //std::cout<< "Point at (u, v) = (" << u << ", " << v << "): " << val[0]<<" "<<val[1]<<" "<<val[2] << endl;
       // std::cout << "u-direction tangent vector (du): "  << endl;
    }
    else {
        // std::cout << "Failed to evaluate the NURBS surface at (u, v) = (" << u << ", " << v << ")." << endl;
    }
    if (d == 0) {
        return du;
    }
    else if (d == 1) {
        return dv;
    }
    else if (d == 2) {
        return duu;
    }
    else if (d == 3) {
        return duv;
    }
    else if (d == 4) {
        return dvv;
    }
}

//判断分割后的曲面是否可能有极点
int ConvexHullOnSplinedSurface(const ON_NurbsSurface& nurbS) {
    const ON_NurbsSurface* nurbSurface = ON_NurbsSurface::Cast(&nurbS);
    int u_count = nurbSurface->CVCount(0);  // 获取 U 方向的控制点数
    int v_count = nurbSurface->CVCount(1);  // 获取 V 方向的控制点数
    ON_3dPoint cv1;
    ON_3dPoint cv2;
    double dv;
    int turnu = 0;
    int turnv = 0;
    //判断四个顶点是否为极点
    int a = nurbSurface->Domain(0).Max();
    int b = nurbSurface->Domain(0).Min();
    int c = nurbSurface->Domain(1).Max();
    int d = nurbSurface->Domain(1).Min();
    int points[4][2] = {
   {a, c},
   { a, d },
   { b,c },
   { b,d }
    };

    // 遍历四个点
    for (int i = 0; i < 4; ++i) {
        double x = points[i][0];
        double y = points[i][1];
        //  // 计算切向量
        Vector3a Su = GetDUVOnNURBSSurface(nurbS, x, y, 0);  // u方向的切向量
        Vector3a Sv = GetDUVOnNURBSSurface(nurbS, x, y, 1);  // v方向的切向量

        // 计算法向量
        Vector3a normal = Su.cross(Sv).normalize();  // 叉积得到法向量并归一化
        if (abs(normal.x) <= 1e-6 && abs(normal.y) <= 1e-6) {
            ON_3dPoint point1 = GetPointOnNURBSSurface(nurbS, x, y);

            // 输出计算出的点的坐标
           /* std::cout << "Iteration at (u=" << x << ", v=" << y << "): "
                << point1.x << ", " << point1.y << ", " << point1.z << std::endl;*/
            return 1;
        }
    }

    // 遍历所有控制点
    for (int i = 0; i < u_count; ++i)
    {
        int turn1 = 0;
        int turn2 = 0;
        nurbSurface->GetCV(i, 0, cv1);
        cv2 = cv1;
        nurbSurface->GetCV(i, 1, cv1);
        dv = cv1.z - cv2.z;
        if (dv > 0) {
            turn1 = 1;
        }
        else if (dv < 0) {
            turn2 = -1;
        }
        for (int j = 2; j < v_count; ++j)
        {
            cv2 = cv1;
            nurbSurface->GetCV(i, j, cv1);
            dv = cv1.z - cv2.z;
            if (dv > 0) {
                turn1 = turn1 + 1;
            }
            else if (dv < 0) {
                turn2 = turn2 - 1;
            }
        }
        if (turn1 * turn2 < 0) {
            turnv = turnv + 1;
        }
    }
    for (int i = 0; i < v_count; ++i)
    {
        int turn1 = 0;
        int turn2 = 0;
        nurbSurface->GetCV(0, i, cv1);
        cv2 = cv1;
        nurbSurface->GetCV(1, i, cv1);
        dv = cv1.z - cv2.z;
        if (dv > 0) {
            turn1 = 1;
        }
        else if (dv < 0) {
            turn2 = -1;
        }
        for (int j = 2; j < u_count; ++j)
        {
            cv2 = cv1;
            nurbSurface->GetCV(j, i, cv1);
            dv = cv1.z - cv2.z;
            if (dv > 0) {
                turn1 = turn1 + 1;
            }
            else if (dv < 0) {
                turn2 = turn2 - 1;
            }
        }
        if (turn1 * turn2 < 0) {
            turnu = turnu - 1;
        }
    }
    if (turnu != 0 || turnv != 0) {
        return 1;
    }
    return 0;
}


//用法向锥判断分割后的曲面是否含有法向为（0,0,1）的点
int NormalCone(const ON_NurbsSurface& nurbS) {
    const ON_NurbsSurface* nurbSurface = ON_NurbsSurface::Cast(&nurbS);

  
    ON_NurbsSurface* surface = const_cast<ON_NurbsSurface*>(nurbSurface);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // int u_count = nurbSurface->CVCount(0);  // 获取 U 方向的控制点数
   // int v_count = nurbSurface->CVCount(1);  // 获取 V 方向的控制点数
   //  // 获取U方向的度数
   // int u_degree = nurbSurface->Degree(0);  // 0表示U方向
   // // 获取V方向的度数
   // int v_degree = nurbSurface->Degree(1);  // 1表示V方向


   // int uKnotCount = nurbSurface->KnotCount(0);  // 0 表示U方向
   //// 获取V方向的节点个数
   // int vKnotCount = nurbSurface->KnotCount(1);  // 1 表示V方向
   // const double* u_knot = nurbSurface->Knot(0);
   // const double* v_knot = nurbSurface->Knot(1);

   // double* U_bezier = new double[uKnotCount + 2];
   // U_bezier[0] = u_knot[0];
   // for (int i = 0; i < uKnotCount; i++) {
   //     U_bezier[i + 1] = u_knot[i];
   //   // std::cout << U_bezier[i + 1] << "， ";
   // }//std::cout << "  \n ";
   // U_bezier[uKnotCount + 1] = u_knot[uKnotCount - 1];
   // double* V_bezier = new double[vKnotCount + 2];
   // V_bezier[0] = v_knot[0];
   // for (int i = 0; i < vKnotCount; i++) {
   //     V_bezier[i + 1] = v_knot[i];
   //     //std::cout << V_bezier[i + 1] << "， ";
   // }
   // V_bezier[vKnotCount + 1] = v_knot[vKnotCount - 1];
   // /*for (int i = 0; i < uKnotCount+2; ++i) {
   //      std::cout << U_bezier[i] << "， ";
   //  }std::cout << "  \n ";
   //  for (int i = 0; i < vKnotCount+2; ++i) {
   //      std::cout << V_bezier[i] << "， ";
   //  }
   //  std::cout << "  \n ";*/
   // double* ConPnt_bezier_1 = new double[3 * u_count * v_count];
   // ON_3dPoint  cv1;
   // for (int i = 0; i < u_count; i++)
   // {
   //     for (int j = 0; j < v_count; j++)
   //     {
   //         nurbSurface->GetCV(i, j, cv1);
   //         for (int k = 0; k < 3; k++)
   //         {
   //             if (k == 0) {
   //                 ConPnt_bezier_1[3 * i * v_count + 3 * j + k] = cv1.x;
   //             }
   //             else if (k == 1) {
   //                 ConPnt_bezier_1[3 * i * v_count + 3 * j + k] = cv1.y;
   //             }
   //             else if (k == 2) {
   //                 ConPnt_bezier_1[3 * i * v_count + 3 * j + k] = cv1.z;
   //             }
   //         }
   //          //std::cout << ConPnt_bezier_1[3 * i * v_count + 3 * j ] << "， " << ConPnt_bezier_1[3 * i * v_count + 3 * j+1] << "， " << ConPnt_bezier_1[3 * i * v_count + 3 * j+2] << "      ";
   //     }  //std::cout << "  \n ";
   // }
   // //SISLSurf* surf_1 = new SISLSurf();
   // SISLSurf* surf_1 = newSurf(u_count, v_count, u_degree + 1, v_degree + 1, U_bezier, V_bezier, ConPnt_bezier_1, 1, 3, 1);
   // // std::cout << "surf_1->ikind" << surf_1->ikind<< "\n ";
   // if (surf_1 == nullptr) {
   //     std::cerr << "Failed to create surface object. newsurf is a null pointer!" << std::endl;
   // }
   // else {
   //     //std::cout << "Surface object created successfully." << std::endl;

   // }

   // int* jstat = new int;/*SISL的控制排列与一般的UV排列方向相反，行是U方向*/
   // if (surf_1 != nullptr) 
   // s1990(surf_1, 1e-6, jstat);
   //  //std::cout << "surf_1->pdir->ecoef." << surf_1->pdir->ecoef[0]<<","<<surf_1->pdir->ecoef[1] << ","<<surf_1->pdir->ecoef[2] << "     " << surf_1->pdir->aang  << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    FLOAT axis[3];
    FLOAT theta;
    get_surface_uv_norm_cone_opennurbs(axis, theta, surface);
   /* std::cout << "axis =" << axis[0] << ", " << axis[1] << ", "
        << axis[2] << "; theta=  " << theta << std::endl;*/
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  // //////////////////////////////////////////////////////////////////////////////////////////////
  // 
//判断法向锥与(0,0,1)or (0,0,-1)是否有交//sisl
//    double tang;	    /* Angel between two vectors.*/
//    int k1;
//    tang = DZERO;
//    tang += surf_1->pdir->ecoef[2];
//    if (tang >= DZERO)  tang = min((double)1.0, tang);
//    else                tang = max((double)-1.0, tang);
//    tang = acos(tang);
//#define PI 3.14159265358979323846
//    if ((tang + surf_1->pdir->aang) < PI && (PI - tang + surf_1->pdir->aang) < PI &&
//        surf_1->pdir->aang < tang && surf_1->pdir->aang < (PI - tang))
//    {
//        /* A simpel case. The two cones and their mirrors
//           are not intersecting.*/
//        return 0;
//    }
//    else
//        //delete  surf_1;  // 别忘了释放内存
//        return 1;
////
//    //判断法向锥与(0,0,1)or (0,0,-1)是否有交//调用函数
    double tang;	    /* Angel between two vectors.*/
    double tang1;
    double tang_1;
    int k1;
    tang = DZERO;
    tang += axis[2];
    if (tang >= DZERO)  tang = min((double)1.0, tang);
    else                tang = max((double)-1.0, tang);
    tang1 = acos(tang);
    tang_1 = acos(-tang);
    /*std::cout << "tang =" << tang << std::endl;*/
#define PI 3.14159265358979323846
    if (tang1 <= theta || tang_1 <= theta) {
        return 1;
    }
    else { return 0; }
   /* if (tang > theta && (tang + theta) < PI) {
        return 0;
    }
    else if (theta >= tang ||tang >= PI- theta) {
        return 1;
    }*/
    
    //if ((tang + theta) < PI && (PI - tang + theta) < PI &&
    //    theta < tang && theta < (PI - tang))
    //{
    //    /* A simpel case. The two cones and their mirrors
    //       are not intersecting.*/
    //    return 0;
    //}
    //else {
    //    //delete  surf_1;  // 别忘了释放内存
    //    return 1;
    //}
};

// 目标函数 f(u, v) = (N_x - 0)^2 + (N_y - 0)^2 + (N_z - 1)^2
double objective_function(double u, double v, const ON_NurbsSurface& nurbS) {
    //  // 计算切向量
    Vector3a Su = GetDUVOnNURBSSurface(nurbS, u, v, 0);  // u方向的切向量
    Vector3a Sv = GetDUVOnNURBSSurface(nurbS, u, v, 1);  // v方向的切向量

    // 计算法向量
    Vector3a normal = Su.cross(Sv).normalize();  // 叉积得到法向量并归一化
    double Nx = normal.x;
    double Ny = normal.y;
    double Nz = normal.z;
    return (Nx - 0) * (Nx - 0) + (Ny - 0) * (Ny - 0);
    // return (Nx - 0) * (Nx - 0) + (Ny - 0) * (Ny - 0) + (Nz - 1) * (Nz - 1);
}

// 梯度下降法迭代(改牛顿迭代)
bool gradient_descent(double& u, double& v, double learning_rate, int max_iterations,
    const ON_NurbsSurface& nurbS, std::vector<double>& tIntersect) {
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 计算梯度（简化为数值差分）
        Vector3a du = GetDUVOnNURBSSurface(nurbS, u, v, 0);  // u方向的切向量
        Vector3a dv = GetDUVOnNURBSSurface(nurbS, u, v, 1);  // v方向的切向量

        // 计算法向量
        Vector3a normal = du.cross(dv).normalize();  // 叉积得到法向量并归一化
        Vector3a duu = GetDUVOnNURBSSurface(nurbS, u, v, 2);
        Vector3a duv = GetDUVOnNURBSSurface(nurbS, u, v, 3);
        Vector3a dvv = GetDUVOnNURBSSurface(nurbS, u, v, 4);
        // 计算海森矩阵
        // 计算目标函数对 u 的一阶导数
        //double df_du, df_dv, d2f_du2, d2f_dv2, d2f_du_dv;


        //df_du = 2 * normal.x * duu.x + 2 * normal.y * duu.y + 2 * (normal.z - 1) * duu.z;
        //df_dv = 2 * normal.x * duv.x + 2 * normal.y * duv.y + 2 * (normal.z - 1) * duv.z;

        //// 计算目标函数对 u 的二阶导数
        //d2f_du2 = 2 * (duu.x * duu.x + duv.x * duv.x);
        //d2f_dv2 = 2 * (duv.x * duv.x);
        //d2f_du_dv = 2 * (duu.x * duv.x);
        //    std::vector<std::vector<double>> H(2, std::vector<double>(2));
            // 填充海森矩阵的元素
            //H[0][0] = d2f_du2; // f''(x, x)
            //H[0][1] = d2f_du_dv; // f''(x, y)
            //H[1][0] = d2f_du_dv; // f''(y, x)
            //H[1][1] = d2f_dv2; // f''(y, y)
            //double detH = H[0][0] * H[1][1] - H[0][1] * H[1][0];
            //std::vector<std::vector<double>> inverse(2, std::vector<double>(2));
            //inverse[0][0] = H[1][1] / detH;
            //inverse[0][1] = -H[0][1] / detH;
            //inverse[1][0] = -H[1][0] / detH;
            //inverse[1][1] = H[0][0] / detH;
        double df_du = (objective_function(u + 1e-6, v, nurbS) - objective_function(u, v, nurbS)) / 1e-6;
        double df_dv = (objective_function(u, v + 1e-6, nurbS) - objective_function(u, v, nurbS)) / 1e-6;
        double p = objective_function(u, v, nurbS);
        // 更新u和v
        //u = u - (inverse[0][0]*df_du+ inverse[0][1]* df_dv);
        //v = v - (inverse[1][0] * df_du + inverse[1][1] * df_dv);
        u -= learning_rate * df_du;
        v -= learning_rate * df_dv;

        // std::cout << "Iteration " << iter + 1 << ": u = " << u << ", v = " << v << std::endl;

        if (objective_function(u, v, nurbS) < 1e-6) {
            // std::cout << "Converged!" << std::endl;
             //ON_3dPoint point1 = GetPointOnNURBSSurface(nurbS, u, v);

             // 输出计算出的点的坐标
            /* std::cout << "Iteration at (u=" << u << ", v=" << v << "): "
                 << point1.x << ", " << point1.y << ", " << point1.z << std::endl;*/
            tIntersect = { u,v };
            //std::cout << "Iteration " << iter + 1 << ": u = " << u << ", v = " << v << std::endl;
            return true;
            break;
        }
    }
    return false;
}


int SplitTheSurface(const ON_NurbsSurface& nurbS, std::vector<std::vector<double>>& para_t) {
    double tol = 1e-4;
    ON::Begin(); // 初始化OpenNURBS库
    const ON_NurbsSurface* nurbSurface = ON_NurbsSurface::Cast(&nurbS);
    int u_count = nurbSurface->CVCount(0);  // 获取 U 方向的控制点数
    int v_count = nurbSurface->CVCount(1);  // 获取 V 方向的控制点数
  // std::cout<< "NormalCone;"<<NormalCone(nurbS)<<"\n";
    int c = NormalCone(nurbS);
    if (NormalCone(nurbS) == 0) {
        return 0;
    }
    if (nurbSurface) {
        double a = nurbSurface->Domain(0).Min() + nurbSurface->Domain(0).Max();
        double b = nurbSurface->Domain(1).Min() + nurbSurface->Domain(1).Max();
        double a0 = nurbSurface->Domain(0).Max() - nurbSurface->Domain(0).Min();
        double b0 = nurbSurface->Domain(1).Max() - nurbSurface->Domain(1).Min();

        ON_Surface* west_side = nullptr;
        ON_Surface* east_side = nullptr;
        ON_NurbsSurface surf_temp, surf_u1;
        if (a0 > b0) {
            double split_param = 0.5 * (a);  // 在曲线的中点处分割
            nurbS.Split(0, split_param, west_side, east_side);
        }
        else {
            double split_param = 0.5 * (b);
            nurbS.Split(1, split_param, west_side, east_side);
        }


        ON_NurbsSurface* west_1 = ON_NurbsSurface::Cast(west_side);
        ON_NurbsSurface* east_1 = ON_NurbsSurface::Cast(east_side);
        surf_temp = *west_1;
        surf_u1 = *east_1;
        std::vector<double> tIntersect;
        double point321[6];
        // std::cout << "domain: " << nurbSurface->Domain(0).Max() << ", " << nurbSurface->Domain(0).Min() << "\n";
         //std::cout << "domainv: " << nurbSurface->Domain(1).Max() << ", " << nurbSurface->Domain(1).Min() << "\n";

       // int p = ConvexHullOnSplinedSurface(nurbS);
        int p = NormalCone(nurbS);
        //  std::cout << "NormalCone1: " << NormalCone(nurbS) << " a0= " << a0 << " b0= " << b0<<"\n";

        if (NormalCone(nurbS) == 1 && a0 < tol && b0 < tol) {

            int max_iterations = 1000;//迭代次数
            double learning_rate = 0.005;
            double u0 = 0.0;
            double v0 = 0.0;
           
         /*   std::vector<std::pair<double, double>> starts = {
     {0.3 * a, 0.3 * b}, {0.7 * a, 0.3 * b},
    {0.3 * a, 0.7 * b}, {0.7 * a, 0.7 * b}, {0.25 * a, 0.5 * b},
    {0.75 * a, 0.5 * b}, {0.5 * a, 0.25 * b},{0.5 * a, 0.5 * b}, {0.5 * a, 0.75 * b}
            };*/

            std::vector<std::pair<double, double>> starts = {
    {0.5 * a, 0.6 * b}
            };
            int rt = 0;
            for (const std::pair<double, double>& p : starts) {
                u0 = p.first;
                v0 = p.second;
                bool test = gradient_descent(u0, v0, learning_rate, max_iterations, nurbS, tIntersect);
                if (test) {
                    bool samepara = true;
                    for (int i = 0; i < para_t.size(); i++) {
                        if (abs(tIntersect[0] - para_t[i][0]) < 1e-4 && abs(tIntersect[1] - para_t[i][1]) < 1e-4) {
                            samepara = false;
                        }
                    }
                    if (abs(tIntersect[0]) > 1 || abs(tIntersect[1]) > 1 || tIntersect[0] < 0 || tIntersect[1] < 0) {
                        samepara = false;
                    }
                    if (samepara) {
                        para_t.push_back(tIntersect);
                    }
                    //std::cout << "Before return" << std::endl;
                    rt = 1;
                }

            }
            return rt;
        }

        if (a0 >= tol || b0 >= tol) {
            /*std::cout << "NormalCone1 surf_temp: " << NormalCone(surf_temp) << "\n";
            std::cout << "NormalCone1 surf_u1: " << NormalCone(surf_u1) << "\n";*/
            if (NormalCone(surf_temp) == 1 && ((west_1->Domain(0).Max() - west_1->Domain(0).Min()) >= tol / 2.0 || (west_1->Domain(1).Max() - west_1->Domain(1).Min()) >= tol / 2.0)) {
                SplitTheSurface(surf_temp, para_t);

            };
            if (NormalCone(surf_u1) == 1 && ((east_1->Domain(0).Max() - east_1->Domain(0).Min()) >= tol / 2.0 || (east_1->Domain(1).Max() - east_1->Domain(1).Min()) >= tol / 2.0)) {

                SplitTheSurface(surf_u1, para_t);

            }

        }

        //// 清理内存
        if (west_side != nullptr) {
            delete west_side;
            west_side = nullptr;  // 清空指针以防止悬挂指针
        }

        if (east_side != nullptr) {
            delete east_side;
            east_side = nullptr;  // 同样清空指针
        }

    }
    else {
        std::cerr << "Failed to extract Nurbs!" << std::endl;
    }
    ON::End();
}




std::vector<Eigen::MatrixXd>  TransformControlPoints(int n, int m, std::vector<Eigen::MatrixXd> Pw, Matrix4x4 transform) {
    vector<Eigen::MatrixXd> Pwt = Pw;
    int cv_count_u = n + 1; // U 方向的控制点数量
    int cv_count_v = m + 1; // V 方向的控制点数量
    for (int i = 0; i < cv_count_u; i++) {
        for (int j = 0; j < cv_count_v; j++) {
            Point p0 = { Pw[0](i, j), Pw[1](i, j),Pw[2](i, j) };
            // 进行变换
            Point point1 = transform.multiply(p0);
            Pwt[0](i, j) = point1.x;
            Pwt[1](i, j) = point1.y;
            Pwt[2](i, j) = point1.z;
        }
    }
    return Pwt;
}


//void FindPointsOnBoundary(const ON_NurbsSurface& nurbs_surf1) {
//   int n = nurbs_surf1.CVCount(0);
//    int m = nurbs_surf1.CVCount(1);
//    int zmin = nurbs_surf1.CV(0, 0)[2];
//    int zmax = nurbs_surf1.CV(0, 0)[2];
//
//    for (int i = 0; i < n; i++) {
//        int j = 0;
//        if (nurbs_surf1.CV(i, j)[2]<zmin) {
//            zmin = nurbs_surf1.CV(i, j)[2];
//        }
//        if (nurbs_surf1.CV(i, j)[2] > zmax) {
//            zmax = nurbs_surf1.CV(i, j)[2];
//        }
//    }
//    if (zmax*zmin<=0)
//}



void FindPointsWithZZero(const ON_NurbsSurface& nurbs_surf, Matrix4x4 transform) {
    // 设置遍历步长
    double step = 0.01; // 参数空间的步长，可以调整精度
    double tolerance = 1e-6; // 容差，用于判断y是否接近0



    //求边界与平面交点
    for (double u = 0.0; u <= 1.0; u += step) {
        double v = 0.0;
        ON_3dPoint point = nurbs_surf.PointAt(u, v);
        Point p0 = { point.x, point.y, point.z };
        // 进行变换
        Point point1 = transform.multiply(p0);
        if (fabs(point1.y) < tolerance) {
            // 输出满足条件的点
            std::cout << "Point with z = 0 found at (u = " << u << ", v = " << v << "): "
                << point.x << ", " << point.y << ", " << point.z << std::endl;
        }
    }

    for (double u = 0.0; u <= 1.0; u += step) {
        double v = 1.0;
        ON_3dPoint point = nurbs_surf.PointAt(u, v);
        Point p0 = { point.x, point.y, point.z };
        // 进行变换
        Point point1 = transform.multiply(p0);
        if (fabs(point1.y) < tolerance) {
            // 输出满足条件的点
            std::cout << "Point with z = 0 found at (u = " << u << ", v = " << v << "): "
                << point.x << ", " << point.y << ", " << point.z << std::endl;
        }
    }

    for (double v = 0.0; v <= 1.0; v += step) {
        double u = 0.0;
        ON_3dPoint point = nurbs_surf.PointAt(u, v);
        Point p0 = { point.x, point.y, point.z };
        // 进行变换
        Point point1 = transform.multiply(p0);
        if (fabs(point1.z) < tolerance) {
            // 输出满足条件的点
            std::cout << "Point with z = 0 found at (u = " << u << ", v = " << v << "): "
                << point.x << ", " << point.y << ", " << point.z << std::endl;
        }
    }

    for (double v = 0.0; v <= 1.0; v += step) {
        double u = 1.0;
        ON_3dPoint point = nurbs_surf.PointAt(u, v);
        Point p0 = { point.x, point.y, point.z };
        // 进行变换
        Point point1 = transform.multiply(p0);
        if (fabs(point1.z) < tolerance) {
            // 输出满足条件的点
            std::cout << "Point with z = 0 found at (u = " << u << ", v = " << v << "): "
                << point.x << ", " << point.y << ", " << point.z << std::endl;
        }
    }
    //// 遍历 u, v 参数空间
    //for (double u = 0.0; u <= 1.0; u += step) {
    //    for (double v = 0.0; v <= 1.0; v += step) {
    //        // 获取曲面上参数 (u, v) 对应的点
    //        ON_3dPoint point = nurbs_surf.PointAt(u, v);

    //        // 检查 y 坐标是否为 0（或者接近 0）
    //        if (fabs(point.z) < tolerance) {
    //            // 输出满足条件的点
    //            std::cout << "Point with y = 0 found at (u = " << u << ", v = " << v << "): "
    //                << point.x << ", " << point.y << ", " << point.z << std::endl;
    //        }
    //    }
    //}



}

double BSplineBasis(int i, int k, double t, std::vector<double>& knots)
{
    /*计算 B-Spline 基函数的值*/
    if (k == 0) {
        return (knots[i] <= t && t < knots[i + 1]) ? 1.0 : 0.0;
    }
    else {
        double denom1 = knots[i + k] - knots[i];
        double denom2 = knots[i + k + 1] - knots[i + 1];
        double term1 = 0, term2 = 0;
        if (denom1 == 0.)
            term1 = 0;
        else
            term1 = ((t - knots[i]) / denom1) * BSplineBasis(i, k - 1, t, knots);
        if (denom2 == 0.)
            term2 = 0;
        else
            term2 = ((knots[i + k + 1] - t) / denom2) * BSplineBasis(i + 1, k - 1, t, knots);

        return term1 + term2;
    }
}

double getPartialDerivative_BSplineBasis(int order, int i, int k, double t, std::vector<double>& knots)
{
    /*compute the order-th PartialDerivative of B-spline basis */
    double val = 0;


    if (order == 0)
    {
        val = BSplineBasis(i, k, t, knots);
    }
    else
    {
        double denom1 = knots[i + k] - knots[i];
        double denom2 = knots[i + k + 1] - knots[i + 1];
        double term1 = 0, term2 = 0;
        if (denom1 == 0.)
            term1 = 0;
        else
            term1 = (k / denom1) * getPartialDerivative_BSplineBasis(order - 1, i, k - 1, t, knots);

        if (denom2 == 0.)
            term2 = 0;
        else
            term2 = (k / denom2) * getPartialDerivative_BSplineBasis(order - 1, i + 1, k - 1, t, knots);

        val = term1 + term2;

    }


    return val;
}
//分别求控制点的x，y，z坐标.c==0输出控制点x坐标，c==1输出y坐标，c==2输出z坐标。
vector<vector<double>> ExtractControlPoints(const ON_NurbsSurface& nurbsSurface, int c) {
    int numU = nurbsSurface.CVCount(0); // U方向控制点数量
    int numV = nurbsSurface.CVCount(1); // V方向控制点数量
    vector<vector<double>> Px;
    vector<double>px;
    //std::cout << "Control Points X Coordinates:" << std::endl;

    for (int i = 0; i < numU; i++) {
        px = {};
        for (int j = 0; j < numV; j++) {
            ON_3dPoint pt;
            if (nurbsSurface.GetCV(i, j, pt) && c == 0) {
                px.push_back(pt.x);
            }
            else if (nurbsSurface.GetCV(i, j, pt) && c == 1) {
                px.push_back(pt.y);
            }
            else if (nurbsSurface.GetCV(i, j, pt) && c == 2) {
                px.push_back(pt.z);
            }
            Px.push_back(px);

        }
    }
    return Px;
}

//计算曲面上的点坐标及其基函数
//void BezierBSplineBasis(const ON_NurbsSurface& nurbs_surf, Point p) {
//
//    const ON_NurbsSurface* nurbSurface = ON_NurbsSurface::Cast(&nurbs_surf);
//    int uKnotCount = nurbs_surf.KnotCount(0);  // 0 表示 u 方向
//    std::vector<double> uKnots;
//    for (int i = 0; i < uKnotCount; i++) {
//        double knot = nurbs_surf.Knot(0, i);
//        uKnots.push_back(knot);
//
//        int vKnotCount = nurbs_surf.KnotCount(1);  // 1 表示 v 方向
//        std::vector<double> vKnots;
//        for (int i = 0; i < vKnotCount; i++) {
//            double knot = nurbs_surf.Knot(1, i);
//            vKnots.push_back(knot);
//        }
//        //先假设控制点权重都为1
//        double numerator_x = 0, numerator_y = 0, numerator_z = 0;
//        double denominator = 0;
//        double u, double v,
//            vector<vector<double>> Px = ExtractControlPoints(nurbs_surf, 0),  // 控制点 x 坐标
//            vector<vector<double>> Py = ExtractControlPoints(nurbs_surf, 1),  // 控制点 y 坐标
//            vector<vector<double>> Pz = ExtractControlPoints(nurbs_surf, 2),  // 控制点 z 坐标
//           //const vector<vector<double>>& weights, // 权重
//            int p = nurbs_surf.Degree(0);   // U 方向的次数
//        // 获取 V 方向的节点向量和次数
//        int q = nurbs_surf.Degree(1);   // V 方向的次数
//        int numU, int numV;// 控制点网格大小
//
//        for (int i = 0; i < numU; i++) {
//            for (int j = 0; j < numV; j++) {
//                double Ni = BSplineBasis(i, p, u, uKnots);
//                double Nj = BSplineBasis(j, q, v, vKnots);
//                //double weight = weights[i][j];
//
//                double R = (Ni * Nj);
//
//                numerator_x += R * Px[i][j];
//                numerator_y += R * Py[i][j];
//                numerator_z += R * Pz[i][j];
//                denominator += R;
//            }
//        }//计算出曲面的基函数
//    }
//}


//自适应步长计算
double ComputeStepSize(const ON_NurbsSurface& nurbs_surf, double u0, double v0)
{
    // 1. 基于曲率的步长控制
    // 曲率越大，步长越小
// 1. 求到二阶偏导，der_count=2，数组长度应为 18
    double der[18];
    nurbs_surf.Evaluate(u0, v0, 2, 3, der);

    // 2. 按顺序取出偏导数据
    ON_3dVector Ds(der[3], der[4], der[5]);    // Su
    ON_3dVector Dt(der[6], der[7], der[8]);    // Sv
    ON_3dVector Dss(der[9], der[10], der[11]);  // Suu
    ON_3dVector Dst(der[12], der[13], der[14]); // Suv
    ON_3dVector Dtt(der[15], der[16], der[17]); // Svv

    // 2. 基于弦高误差的步长控制
    // 步长 h 满足：h^2 * curvature / 8 < Fleche
    double aStep = 0;
    // 1. 法向量
    ON_3dVector N = ON_CrossProduct(Ds, Dt);
    double n_len = N.Length();
    if (n_len < 1e-12) {
        // 退化点，切向量共线，无法定义法向
        return 0.0;
    }
    N /= n_len;
    // 2. 第一基本形式系数
    double E = Ds * Ds;
    double F = Ds * Dt;
    double G = Dt * Dt;

    // 3. 第二基本形式系数
    double L = N * Dss;
    double M = N * Dst;
    double Nn = N * Dtt;   // 避免与法向 N 重名

    // 4. 分母（判别式）
    double denom = E * G - F * F;
    if (std::abs(denom) < 1e-12) {
        return 0.0;
    }

    // 5. 高斯曲率 K 与平均曲率 H
    double K = (L * Nn - M * M) / denom;
    double H = (G * L + E * Nn - 2.0 * F * M) / (2.0 * denom);

    // 6. 主曲率 k1, k2
    double disc = H * H - K;
    if (disc < 0.0) disc = 0.0;   // 数值误差保护
    double sqrt_disc = std::sqrt(disc);

    double k1 = H + sqrt_disc;
    double k2 = H - sqrt_disc;
    // 7. 最大曲率（取绝对值最大）
    double Fleche = 0.0001;//(线段与真实曲线/曲面之间的最大垂直距离)

    double aMaxCurvature = max(std::abs(k1), std::abs(k2));
        aStep = sqrt(8.0 * Fleche / aMaxCurvature);
 

    // 5. 限制步长范围
    aStep = max(aStep, 1e-6);  // 最小步长
    aStep = min(aStep, 0.1);   // 最大步长

    return aStep;
}

//trace函数是最后通过一个交点trace出一条线
//p为所求交点坐标
vector<double> traceline(const ON_NurbsSurface& nurbs_surf, double u0, double v0, int dir) {
    const double epsilon = 1e-3;  // 收敛精度(设计自适应精度)
    // 计算曲面一阶偏导数
    ON_3dVector dU, dV;
    double point[9];
    // 计算u方向的一阶偏导数
    nurbs_surf.Evaluate(u0, v0, 1, 3, point);
    double fx = point[5];
    double fy = point[8];
    

    // 计算v方向的一阶偏导数
  //  double fy = nurbs_surf.Evaluate(u0, v0, 1, 1, point);


    double h = 0.01;//步长(设计自适应步长)
 
 //   h = ComputeStepSize(nurbs_surf, u0, v0);
    double k = h / sqrt(fx * fx + fy * fy);
    vector<double> p0 = { k * (-fy), k * (fx) };
    vector<double> p1 = { k * (-fy), k * (fx) };
    if (dir == -1) {
        p0[0] *= dir;
        p0[1] *= dir;
        p1[0] *= dir;
        p1[1] *= dir;
    }
    ON_3dPoint f0 = GetPointOnNURBSSurface(nurbs_surf, u0 + p0[0], v0 + p0[1]);
    double w0 = f0.z / ((-k * fx * fx) - k * fy * fy);
  double norm = 1;


    for (int i = 0; norm > epsilon && abs(f0.z) > epsilon; i++) {
        p0 = p1;
        //std::cout << "cos= " << cos(w0) << "\n";
        //std::cout << "p0= " << p0[1] <<", " << p0[1] << "\n";
        p1[0] = cos(w0) * p0[0] + sin(w0) * p0[1];
        p1[1] = -sin(w0) * p0[0] + cos(w0) * p0[1];
        f0 = GetPointOnNURBSSurface(nurbs_surf, u0 + p1[0], v0 + p1[1]);
        nurbs_surf.Evaluate(u0 + p1[0], v0 + p1[1], 1, 3, point);
        fx = point[5];
        fy = point[8];
        w0 = f0.z / ((-p1[1] * fx) + p1[0] * fy);
        norm = sqrt((p1[0] - p0[0]) * (p1[0] - p0[0]) + (p1[1] - p0[1]) * (p1[1] - p0[1]));
   
    }
    p1 = { p1[0] + u0, p1[1] + v0 };
    return p1;

}

//trace函数是最后通过一个交点trace出一条线
//p为所求交点坐标
vector<double> tracelineloop(const ON_NurbsSurface& nurbs_surf, double u0, double v0, int dir) {
    const double epsilon = 1e-9;  // 收敛精度
    // 计算曲面一阶偏导数
    ON_3dVector dU, dV;
    double point[9];
    // 计算u方向的一阶偏导数
    nurbs_surf.Evaluate(u0, v0, 1, 3, point);
    
    double fx = point[5];
    double fy = point[8];

    // 计算v方向的一阶偏导数
  //  double fy = nurbs_surf.Evaluate(u0, v0, 1, 1, point);


    double h = 0.005;//步长
    /////////////////////////////////////
    ///////////////////////////////////////
   /// 自适应步长
    ON_3dPoint point1;
    ON_3dVector Du, Dv;        // 一阶偏导
    ON_3dVector Duu, Duv, Dvv; // 二阶偏导

    if (nurbs_surf.Ev2Der(u0, v0, point1, Du, Dv, Duu, Duv, Dvv)) {
        // 成功获取
    }
    // 法向量 (注意单位化)
    ON_3dVector N = ON_CrossProduct(Du, Dv);


    // 第一基本形式系数
    double E = Du * Du;
    double F = Du * Dv;
    double G = Dv * Dv;

    // 第二基本形式系数
    double L = Duu * N;
    double M = Duv * N;
    double Nn = Dvv * N; // 避免与法向量 N 重名
    double denom = E * G - F * F; // 判别式


    // 高斯曲率 K = det(II) / det(I)
    double K = (L * Nn - M * M) / denom;

    // 平均曲率 H = (k1 + k2) / 2
    // 利用展开式: 2H = (E*Nn - 2*F*M + G*L) / (E*G - F^2)
    double H = (E * Nn - 2.0 * F * M + G * L) / (2.0 * denom);

    // 通过 K 和 H 解出主曲率 k1, k2
    double discriminant = H * H - K;
    if (discriminant < 0) discriminant = 0; // 数值误差保护



        // 1. 取 z 分量
    double zu = Du.z;
    double zv = Dv.z;
    double zuu = Duu.z;
    double zuv = Duv.z;
    double zvv = Dvv.z;

    double grad2 = zu * zu + zv * zv;

    // 2. 参数域切向（归一化）
    double norm = sqrt(grad2);
    double up = -zv / norm;   // u'
    double vp = zu / norm;   // v'

    // 3. 二阶导右端项 D
    double D = zuu * up * up
        + 2.0 * zuv * up * vp
        + zvv * vp * vp;

    // 4. 解 u'', v''
    // 方程组:
    //   zu*u'' + zv*v'' = -D
    //   up*u'' + vp*v'' = 0
    double det = zu * vp - zv * up;


    double upp = (-D * vp) / det;   // u''
    double vpp = (D * up) / det;   // v''

    // 5. 三维一阶、二阶导
    ON_3dVector Sp = Du * up + Dv * vp;

    ON_3dVector Spp = Duu * (up * up)
        + Duv * (2.0 * up * vp)
        + Dvv * (vp * vp)
        + Du * upp
        + Dv * vpp;

    // 6. 取 xy 分量算曲率
    double dx = Sp.x, dy = Sp.y;
    double ddx = Spp.x, ddy = Spp.y;

    double speed2 = dx * dx + dy * dy;


    double cross = dx * ddy - dy * ddx;






    double sqrt_disc = sqrt(discriminant);
    double k1 = H + sqrt_disc;
    double k2 = H - sqrt_disc;
   /////// double kappa = max(fabs(k1), fabs(k2));///////最大曲率绝对值
    double     kappa = fabs(cross) / pow(speed2, 1.5);/////////交线曲率
 //////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    h = sqrt(8 * epsilon / kappa);

    double k = h / sqrt(fx * fx + fy * fy);
    vector<double> p0 = { k * (-fy), k * (fx) };
    vector<double> p1 = { k * (-fy), k * (fx) };
    if (dir == -1) {
        p0[0] *= dir;
        p0[1] *= dir;
        p1[0] *= dir;
        p1[1] *= dir;
    }
    ON_3dPoint f0 = GetPointOnNURBSSurface(nurbs_surf, u0 + p0[0], v0 + p0[1]);
    double w0 = f0.z / ((-k * fx * fx) - k * fy * fy);
   norm = 1;
    int times = 0;

    for (int i = 0; norm > epsilon && abs(f0.z)> epsilon; i++) {
        p0 = p1;
        //std::cout << "cos= " << cos(w0) << "\n";
        //std::cout << "p0= " << p0[1] <<", " << p0[1] << "\n";
        p1[0] = cos(w0) * p0[0] + sin(w0) * p0[1];
        p1[1] = -sin(w0) * p0[0] + cos(w0) * p0[1];
        f0 = GetPointOnNURBSSurface(nurbs_surf, u0 + p1[0], v0 + p1[1]);
        nurbs_surf.Evaluate(u0 + p1[0], v0 + p1[1], 1, 3, point);
        fx = point[5];
        fy = point[8];
        w0 = f0.z / ((-p1[1] * fx) + p1[0] * fy);
        norm = sqrt((p1[0] - p0[0]) * (p1[0] - p0[0]) + (p1[1] - p0[1]) * (p1[1] - p0[1]));
        times = i;
        if (i>=1000) {
            vector<double> singlepoint = { u0,v0 };
            return singlepoint;
        }
    }
    p1 = { p1[0] + u0, p1[1] + v0 };
    return p1;

}

//判断边界上的起始交点是否被trace过
//通过判断边界交点是否在交线内
int whethertraced(vector<std::vector<Point>>viewoutput, Point point_ans) {
    const double epsilon = 1e-6;  // 收敛精度
    int i;
    int j;
    Point pointOut;
    for (i = 0; i < viewoutput.size(); i++) {
        for (j = 0; j < viewoutput[i].size(); j++) {
            pointOut = viewoutput[i][j];
            double dis = abs(pointOut.x - point_ans.x) + abs(pointOut.y - point_ans.y) + abs(pointOut.z - point_ans.z);
            if (dis < epsilon) {
                return 1;//该点已经被trace过了
            }
        }
    }
    return 0;

}


//判断环上的交点是否到头
int whetherlooped(vector<double> p1, double p10, double p11) {
    const double epsilon = 1e-6;  // 收敛精度
    
            double dis = abs(p1[0] - p10) + abs(p1[1] - p11);
            if (dis < epsilon) {
                return 1;//该点已经被trace过了
            }
   
    return 0;

}

//判断边界曲线是否整条落在平面上
int whetherlineontheplane(ON_Curve* iso_curve1, double tol) {
    ON_NurbsCurve* nc_ptr = ON_NurbsCurve::Cast(iso_curve1);
    if (!nc_ptr)
    {
        delete iso_curve1;
        return 0;
    }

    ON_NurbsCurve& nc = *nc_ptr;
   
    if (!nc.MakePiecewiseBezier()) return false;

    for (int i = 0; i < nc.CVCount(); i++)
    {
        ON_3dPoint P;
        nc.GetCV(i, P);
        if (fabs(P.z) > tol) return 0;
    }
    return 1;

}
//输出整条曲线
void outputlineontheplane(ON_Curve* iso_curve1, vector<std::vector<Point>>& viewopencurve) {
    ON_Curve* crv = iso_curve1;
    ON_Interval dom = crv->Domain();      // 参数域 [t0, t1]
    int N = 1000;                          // 采样段数
    vector<Point> pts;
   pts.reserve(N + 1);

    for (int i = 0; i <= N; i++)
    {
        double t = dom.ParameterAt((double)i / N);  // 均匀参数采样
        ON_3dPoint P = crv->PointAt(t);
        Point pt;
        pt.x = P.x;
        pt.y = P.y;
        pt.z = P.z;  

        pts.push_back(pt);
    }
    viewopencurve.push_back(pts);
    return;
}



//int delthepoint(vector<vector<double>> para_uv) {
//    const double epsilon = 1e-3;  // 收敛精度
//    int delnumber = 0;
//    for (int i = 0; i < para_uv.size()-1; i++) {
//        for (int j = i+1; j < para_uv.size(); j++) {
//            double dis = abs(para_uv[i][0] - para_uv[j][0])* abs(para_uv[i][0] - para_uv[j][0]) + abs(para_uv[i][1] - para_uv[j][1])* abs(para_uv[i][1] - para_uv[j][1]);
//            if (dis < epsilon) {
//                para_uv.erase(para_uv.begin() + j);
//                delnumber = delnumber + 1;
//            }
//        }
//    }
//    return delnumber;
//}


void Boundaryintersection(int curveNumber,ON_Curve* iso_curve1, vector<double>& para_1, double point234[6], vector<std::vector<Point>>& viewoutput, ON_NurbsSurface nurbs_surf,vector<int> values, ON_NurbsSurface nurbs_surf1, std::vector<Point>&  Boundary1, double minUknot, double minVknot, double maxUknot, double maxVknot){
        SplitTheBoundary(iso_curve1, para_1);

        for (int i = 0; i < para_1.size(); i++) {
            iso_curve1->Evaluate(para_1[i], 1, 3, point234);
            if (abs(point234[2]) < 1e-4) {
                /* std::cout << "" << point234[0] << " " << point234[1] << " " << point234[2] << "";
                 std::cout << "\n";*/
                Point point_ans = { point234[0] ,point234[1] ,point234[2] };
                if (whethertraced(viewoutput, point_ans) == 0) {
                    //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                    ON_3dPoint original_point432;
                    if (curveNumber==1) {
                        original_point432 = GetPointOnNURBSSurface(nurbs_surf, para_1[i], 0);
                    }
                    if (curveNumber == 2) {
                         original_point432 = GetPointOnNURBSSurface(nurbs_surf, para_1[i], 1);
                    }
                    if (curveNumber == 3) {
                       original_point432 = GetPointOnNURBSSurface(nurbs_surf, 0, para_1[i]);
                    }
                    if (curveNumber == 4) {
                        original_point432 = GetPointOnNURBSSurface(nurbs_surf, 1,para_1[i]);
                    }
                    
                    Point original_point = { original_point432[0] ,original_point432[1] ,original_point432[2] };
                    /* cout << "逆变换后的点: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                      std::cout << "trace出的点： " << "\n";*/
                    for (int dir : values) {
                        vector<double> p1;
                        if (curveNumber == 1) {
                            p1 = traceline(nurbs_surf1, para_1[i], 0, dir);
                        }
                        if (curveNumber == 2) {
                            p1 = traceline(nurbs_surf1, para_1[i], 1, dir);
                        }
                        if (curveNumber == 3) {
                            p1 = traceline(nurbs_surf1,0, para_1[i], dir);
                        }
                        if (curveNumber == 4) {
                            p1 = traceline(nurbs_surf1, 1,para_1[i], dir);
                        }
                       

                        Boundary1.push_back(original_point);
                        int numbertraced = 0;
                        for (int j = 0; j < 1000 && p1[0] <= maxUknot && p1[1] <= maxVknot && p1[0] >= minUknot && p1[1] >= minVknot; j++) {//每一个点trace出来的点数

                            ON_3dPoint point432 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
                            //            std::cout << "ControlPoint" <<"\n";
                            //            ON_3dPoint point1;
                            //for (int i = 0; i < nurbs_surf1.CVCount(0); i++) {
                            //    for (int j = 0; j < nurbs_surf1.CVCount(1); j++) {
                            //        nurbs_surf1.GetCV(i, j, point1);
                            //        std::cout << point1.x << ",  " << point1.y << " , " << point1.z << ",    ";
                            //    }
                            //    //std::cout << "\n";
                            //}
                           // std::cout << "\n"; 
                                       // cout << "p1[0]=" << p1[0] << "  p1[1]=" << p1[1]<<endl;
                                       // if (abs(point432[2]) < 1e-4) {
                                           // std::cout << "(" << point432[0] << "," << point432[1] << "," << point432[2] << ")";
                                           // std::cout << "\n";
                            Point point_ans = { point432[0] ,point432[1] ,point432[2] };

                            Boundary1.push_back(point_ans);

                            //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                            //Point original_point = point_ans;
                           // cout << "trace的点逆变换后为: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                        //***** cout << point_ans.x << " " << point_ans.y << " " << point_ans.z << " " << endl;
                           // cout << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                            //}
                            //else {
                              //  cout << "所求点不满足要求。" << endl;
                            //}
                            p1 = traceline(nurbs_surf1, p1[0], p1[1], dir);
                            numbertraced = j;
                        }
                        // int Bsize = Boundary1.size();
                        viewoutput.push_back(Boundary1);
                        Boundary1.clear();
                    }
                }
            }
        }

    }


// 把 RGB 像素写成 24 位 BMP
bool writeBMP(const std::string& filename,
    const std::vector<unsigned char>& img,
    int W, int H) {
    std::ofstream f(filename, std::ios::binary);
    if (!f) return false;

    int rowSize = (W * 3 + 3) & ~3;  // 每行按 4 字节对齐
    int dataSize = rowSize * H;
    int fileSize = 54 + dataSize;

    // BMP 文件头 (14 字节)
    unsigned char fileHeader[14] = {};
    fileHeader[0] = 'B';
    fileHeader[1] = 'M';
    *(int*)&fileHeader[2] = fileSize;
    *(int*)&fileHeader[10] = 54;

    // DIB 头 (40 字节)
    unsigned char infoHeader[40] = {};
    *(int*)&infoHeader[0] = 40;
    *(int*)&infoHeader[4] = W;
    *(int*)&infoHeader[8] = H;
    *(short*)&infoHeader[12] = 1;   // 平面数
    *(short*)&infoHeader[14] = 24;  // 位深
    *(int*)&infoHeader[20] = dataSize;

    f.write((char*)fileHeader, 14);
    f.write((char*)infoHeader, 40);

    // 像素数据：BMP 是 BGR 顺序，且从下到上
    std::vector<unsigned char> row(rowSize, 0);
    for (int y = H - 1; y >= 0; --y) {
        for (int x = 0; x < W; ++x) {
            int src = (y * W + x) * 3;
            row[x * 3 + 0] = img[src + 2]; // B
            row[x * 3 + 1] = img[src + 1]; // G
            row[x * 3 + 2] = img[src + 0]; // R
        }
        f.write((char*)row.data(), rowSize);
    }
    return true;
}


// ================= 主函数 =================
  // 输入：多组 3D 点
  // 行为：按 XY 平面投影，画点，保存 PNG 到 ./output/
  // 返回：保存的完整文件路径
std::string savePointsImage(const std::vector<std::vector<Point>>& groups, int POINT_RADIUS,int W,int H) {
    // 白色背景
    std::vector<unsigned char> img(W * H * 3, 255);
    double MARGIN = 0.05;
    // 1. 计算 XY 包围盒
    double minX = 1e300, minY = 1e300;
    double maxX = -1e300, maxY = -1e300;
    for (const auto& group : groups) {
        for (const auto& p : group) {
            minX = min(minX, p.x);
            maxX = max(maxX, p.x);
            minY = min(minY, p.y);
            maxY = max(maxY, p.y);
        }
    }

    double spanX = maxX - minX;
    double spanY = maxY - minY;
    if (spanX < 1e-12) spanX = 1.0;
    if (spanY < 1e-12) spanY = 1.0;

    // 2. 等比缩放 + 居中
    double scaleX = (W * (1 - 2 * MARGIN)) / spanX;
    double scaleY = (H * (1 - 2 * MARGIN)) / spanY;
    double scale = min(scaleX, scaleY);

    double offsetX = (W - spanX * scale) / 2.0;
    double offsetY = (H - spanY * scale) / 2.0;

    // 3. 颜色表
    const std::array<std::array<unsigned char, 3>, 8> colors = { {
        {255, 0, 0}, {0, 128, 0}, {0, 0, 255}, {255, 140, 0},
        {128, 0, 128}, {0, 160, 160}, {200, 0, 100}, {80, 80, 80}
    } };

    // 4. 逐组画点
    for (size_t gi = 0; gi < groups.size(); ++gi) {
        const auto& c = colors[gi % colors.size()];
        for (const auto& p : groups[gi]) {
            int px = static_cast<int>((p.x - minX) * scale + offsetX);
            int py = static_cast<int>(H - ((p.y - minY) * scale + offsetY));
           
            // 在图像上画一个实心圆点
                for (int dy = -POINT_RADIUS; dy <= POINT_RADIUS; ++dy) {
                    for (int dx = -POINT_RADIUS; dx <= POINT_RADIUS; ++dx) {
                        if (dx * dx + dy * dy > POINT_RADIUS * POINT_RADIUS) continue;
                        int x = px + dx;
                        int y = py + dy;
                        if (x < 0 || x >= W || y < 0 || y >= H) continue;
                        int idx = (y * W + x) * 3;
                        img[idx] = c[0];
                        img[idx + 1] = c[1];
                        img[idx + 2] = c[2];
                    
                }
            }
        }
    }
    // 改成你已有的文件夹路径
    std::string OUTPUT_DIR = "C:/Users/lenovo/Desktop/traceMap";
    // 6. 保存
    // 生成带时间戳的文件名
   
    std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif

    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);

    std::string fullPath = OUTPUT_DIR + "/" + std::string("points_") + buf + ".png";
    writeBMP(fullPath, img, W, H);

    return fullPath;
}

static PK_ERROR_code_t create_bsurf_from_onnurbs(const ON_NurbsSurface& nurbs, PK_BSURF_t& bsurf)
{
    
    if (nurbs.IsRational()) {
        return PK_ERROR_unsuitable_entity; // splinewise 不支持有理曲面
    }

    int n_u = nurbs.CVCount(0);
    int n_v = nurbs.CVCount(1);

    // positions 按 u 变化最快的顺序排列
    std::vector<PK_VECTOR_t> positions(n_u * n_v);
    for (int j = 0; j < n_v; ++j) {
        for (int i = 0; i < n_u; ++i) {
            ON_3dPoint pt;
            nurbs.GetCV(i, j, pt);
            positions[i + j * n_u].coord[0] = pt.x;
            positions[i + j * n_u].coord[1] = pt.y;
            positions[i + j * n_u].coord[2] = pt.z;
        }
    }

    PK_BSURF_splinewise_sf_t sf;
    std::memset(&sf, 0, sizeof(sf));

    sf.u_degree = nurbs.Degree(0);
    sf.v_degree = nurbs.Degree(1);
    sf.n_u_positions = n_u;
    sf.n_v_positions = n_v;
    sf.positions = positions.data();

    // 让 Parasolid 自动计算节点向量（最稳妥，避免 OpenNURBS 与 Parasolid 节点格式不匹配）
    sf.u_how_knot = PK_PARAM_knot_auto_c;
    sf.v_how_knot = PK_PARAM_knot_auto_c;
    // u_knot / v_knot 在 auto 模式下不应被使用，保持 nullptr 或 memset 后的零值

    sf.u_end = PK_PARAM_end_natural_c;
    sf.v_end = PK_PARAM_end_natural_c;

    return PK_BSURF_create_splinewise(&sf, &bsurf);
}