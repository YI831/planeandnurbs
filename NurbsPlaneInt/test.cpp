#include <iostream>
#include "opennurbs.h"

#include <iostream>
#include <fstream>
#include "test.h"
#include <fstream>
#include<Geom_BSplineCurve.hxx>
#include<Geom_BSplineSurface.hxx>
#include <IntPatch_Intersection.hxx>
#include <IntPatch_WLine.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <Adaptor3d_TopolTool.hxx>
#include <IntPatch_PrmPrmIntersection.hxx>
#include<IntPolyh_Intersection.hxx>
#include "parasolid_kernel.h"

using namespace Eigen;
using namespace std;

vector<std::vector<Point>> NurbsPlaneInt(double planA, double planB, double planC, float plane_normal[3], ON_NurbsSurface nurbs_surf)
{


    std::vector<std::vector<Point>> viewoutput;
    ///////////////////////////////////////////////
    // ////////////////////////////////////////////
    //std::ifstream SURF;
    //SURF.open("D:\\安装文件\\project\\example\\B-spline_SelfIntFree_1.txt");//输入文件
    ////SURF.open("D:\\安装文件\\project\\example\\曲面数据\\B-spline_Boat.txt");//输入文件
    // //SURF.open("D:\\安装文件\\project\\example\\spline.txt");


    // //degree:(p,q)
    int n, m, p, q;

    // 获取阶数
    int degree_u = nurbs_surf.Degree(0);
    int degree_v = nurbs_surf.Degree(1);
    // 获取控制点数
    int cv_count_u = nurbs_surf.CVCount(0);
    int cv_count_v = nurbs_surf.CVCount(1);

    // 计算 n, m (根据NURBS标准定义)
    n = cv_count_u - 1;  // 最后一个控制点的U索引
    m = cv_count_v - 1;  // 最后一个控制点的V索引
    p = degree_u;
    q = degree_v;
    //SURF >> n;
    //SURF >> m;
    //SURF >> p;
    //SURF >> q;
    ///*  std::cout << "p: " << p << "\n";
    //  std::cout << "q: " << q << "\n";*/



    //double con = 1;
    //int rr1 = 1, rr2 = 1;
    ////int rr1 = 3, rr2 = 5;

    ////Knot :KnotU,KnotV
  /*  std::vector<double> KnotU;
    std::vector<double> KnotV;*/
    // 获取U方向节点
    int u_count = nurbs_surf.KnotCount(0) + 2;
    std::vector<double> KnotU(u_count);
    KnotU[0] = 0;
    KnotU[u_count - 1] = 1;
    for (int i = 0; i < (u_count - 2); i++) {
        KnotU[i + 1] = nurbs_surf.Knot(0, i);
    }

    // 获取V方向节点
    int v_count = nurbs_surf.KnotCount(1) + 2;
    std::vector<double> KnotV(v_count);
    KnotV[0] = 0;
    KnotV[v_count - 1] = 1;
    for (int i = 0; i < (v_count - 2); i++) {
        KnotV[i + 1] = nurbs_surf.Knot(1, i);
    }
    //for (int i = 0; i <= n - p - rr1 + 2; i++)
    //{
    //    double val;
    //    SURF >> val;
    //    KnotU.push_back(val);
    //}
    //for (int i = 0; i <= m - q - rr2 + 2; i++)
    //{
    //    double val;
    //    SURF >> val;
    //    KnotV.push_back(val);
    //}
    ///*for (int i = 0; i <= 114; i++)
    //{
    //    double val;
    //    SURF >> val;
    //    KnotV.push_back(val);
    //}*/
    ////multiplicity :mul_u,mul_v
    std::vector<double> mul_u;
    std::vector<double> mul_v;
    /////////////////////////////////////////////////////////////////////////提取节点向量重数


    std::vector<double> distinct_knots_u;
    std::vector<double> distinct_knots_v;
    // 获取第一个节点
    double current_knot = nurbs_surf.Knot(0, 0);
    distinct_knots_u.push_back(current_knot);

    int count = 1;

    // 遍历所有节点
    for (int i = 1; i < u_count; i++) {
        double knot = nurbs_surf.Knot(0, i);

        if (fabs(knot - current_knot) < ON_ZERO_TOLERANCE) {
            count++;
        }
        else {
            // 保存前一个节点的重数
            mul_u.push_back(static_cast<double>(count));

            // 开始新的节点
            current_knot = knot;
            distinct_knots_u.push_back(current_knot);
            count = 1;
        }
    }

    // 保存最后一个节点的重数
    mul_u.push_back(static_cast<double>(count));


    // 提取V方向节点向量和重数




    current_knot = nurbs_surf.Knot(1, 0);
    distinct_knots_v.push_back(current_knot);

    count = 1;

    for (int i = 1; i < v_count; i++) {
        double knot = nurbs_surf.Knot(1, i);

        if (fabs(knot - current_knot) < ON_ZERO_TOLERANCE) {
            count++;
        }
        else {
            mul_v.push_back(static_cast<double>(count));
            current_knot = knot;
            distinct_knots_v.push_back(current_knot);
            count = 1;
        }
    }

    mul_v.push_back(static_cast<double>(count));
    //////////////////////////////////////////////////////////////////////////////////////////////////

        //for (int i = 0; i < KnotU.size(); i++)
        //{
        //    double val;
        //    SURF >> val;
        //    mul_u.push_back(val);
        //}
        //for (int i = 0; i < KnotV.size(); i++)
        //{
        //    double val;
        //    SURF >> val;
        //    mul_v.push_back(val);
        //}

        //Eigen::MatrixXd ControlPnt_PX(n + 1, m + 1);
        //Eigen::MatrixXd ControlPnt_PY(n + 1, m + 1);
        //Eigen::MatrixXd ControlPnt_PZ(n + 1, m + 1);
        ////Control points: Pw
        //std::vector<Eigen::MatrixXd> Pw;
        //for (int i = 0; i <= n; i++)
        //{
        //    for (int j = 0; j <= m; j++)
        //    {
        //        double pnt0, pnt1, pnt2;
        //        SURF >> pnt0;
        //        SURF >> pnt1;
        //        SURF >> pnt2;
        //        ControlPnt_PX(i, j) = con * pnt0;
        //        ControlPnt_PY(i, j) = con * pnt1;
        //        ControlPnt_PZ(i, j) = con * pnt2;
        //    }
        //}


        //Pw.push_back(ControlPnt_PX);
        //Pw.push_back(ControlPnt_PY);
        //Pw.push_back(ControlPnt_PZ);


        ///* std::cout << "KnotU: " << "\n";
        // for (int i = 0; i < KnotU.size(); i++)
        // {

        //     std::cout << KnotU[i] << " ";

        // }
        // std::cout << "\n";
        // std::cout << "KnotV: " << "\n";
        // for (int i = 0; i < KnotV.size(); i++)
        // {

        //     std::cout << KnotV[i] << " ";

        // }
        // std::cout << "\n";
        // std::cout << "mul_u: " << "\n";
        // for (int i = 0; i < KnotU.size(); i++)
        // {

        //     std::cout << mul_u[i] << " ";

        // }
        // std::cout << "\n";
        // std::cout << "mul_v: " << "\n";
        // for (int i = 0; i < KnotV.size(); i++)
        // {

        //     std::cout << mul_v[i] << " ";

        // }
        // std::cout << "\n";*/
        // /* std::cout << "Pw: " << "\n";
        //  for (int i = 0; i <= n; i++)
        //  {
        //      for (int j = 0; j <= m; j++)
        //      {
        //          std::cout << Pw[0](i, j) << " " << Pw[1](i, j) << " " << Pw[2](i, j) << "   ";
        //      }
        //      std::cout << " " << "\n";
        //  }
        //  std::cout << "\n";*/

        //  //KnotVector :U,V
    std::vector<double> U;
    std::vector<double> V;
    double minUknot = KnotU[0];
    double maxUknot = KnotU[KnotU.size() - 1];
    double minVknot = KnotV[0];
    double maxVknot = KnotV[KnotV.size() - 1];

    for (int i = 0; i < KnotU.size(); i++)
    {

        U.push_back(KnotU[i]);

    }
    for (int i = 0; i < KnotV.size(); i++)
    {

        V.push_back(KnotV[i]);

    }
    ///* std::cout << "U: " << "\n";
    // for (int i = 0; i < U.size(); i++)
    // {

    //     std::cout << U[i] << " ";

    // }
    // std::cout << "\n";
    // std::cout << "V: " << "\n";
    // for (int i = 0; i < V.size(); i++)
    // {

    //     std::cout << V[i] << " ";

    // }
    // std::cout << "\n";*/


    //time_t TotalTime_begin, TotalTime_end, Determine_1_begin, Determine_1_end, B_spline_begin, B_spline_end, Bezier_begin, Bezier_end;
    //TotalTime_begin = clock();
    //std::vector<double> Int_Pnts;
    //std::vector<double> Start_Pnts;


//////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////读取曲面部分


   /* double planA = 0;
    double planB = 0;
    double planC = -2;*/



    /*使用Opennurbs计算NURBS曲面nurbs_surf上的点及k阶偏导
    double val[18];
    nurbs_surf.Evaluate(para_u, para_v, k, 3, val);*/

    //####################################################
    //######################################################假设平面的法向量和一个平面上的点
    //***************************************************
    //Vector3d normal(A, B, C);
    //Vector3d point(a, b, c);
    double A = plane_normal[0];
    double B = plane_normal[1];
    double C = plane_normal[2];
    Vector3d normal(A, B, C);
    Vector3d point(planA, planB, planC);
    //double firstZ = point[2];

    // 计算仿射变换矩阵
    Matrix4d affineMatrix = computeAffineMatrix(normal, point);




    //std::cout << "Affine Transformation Matrix:\n" << affineMatrix << std::endl;

    //// 原始平面上的一个点
    //Point3D p0 = { 1.0, 1.0, 3.0};
    ////检测点是否在平面上
    //if ((a * p0.x + b * p0.y + c * p0.z+d) !=0) {
    //    std::cout << "Point is not on the plant " << std::endl;
    //}

    //// 生成旋转矩阵
    //vector<vector<double>> rotationMatrix = generateRotationMatrix(a, b, c);

    //// 计算平移量，使平面移动到 z=0 位置
    //double translationZ = d / c;
    //vector<double> translation = { 0, 0, -translationZ };

    //// 对点进行仿射变换
    //Point3D transformedPoint = affineTransform(p0, rotationMatrix, translation);

    // 输出结果
  /*  std::cout << "原始点: (" << p0.x << ", " << p0.y << ", " << p0.z << ")" << endl;
    std::cout << "变换后的点: (" << transformedPoint.x << ", " << transformedPoint.y << ", " << transformedPoint.z << ")" << endl;*/

    // 定义原始点
    Point p0 = { 1.0, 0.0, 0.0 };
    Matrix4x4 transform = {
       {affineMatrix
       }
    };


    //// 进行变换
    //Point transformedPoint = transform.multiply(p0);

    // 输出结果
   /* cout << "Original Point: (" << p0.x << ", " << p0.y << ", " << p0.z << ")" << endl;
    cout << "Transformed Point: (" << transformedPoint.x << ", " << transformedPoint.y << ", " << transformedPoint.z << ")" << endl;*/





    ON::Begin(); // 初始化OpenNURBS库
   // ON_NurbsSurface nurbs_surf;
    ON_NurbsSurface nurbs_surf1;
    /*  CreatNURBS(p, q, n, m, Pw, U, V, nurbs_surf);*/

      /*Normalize control point*/
    ON_3dPointArray control_points;
    for (int i = 0; i < nurbs_surf.CVCount(0); i++)
    {
        for (int j = 0; j < nurbs_surf.CVCount(1); j++)
        {
            ON_3dPoint control_point;
            nurbs_surf.GetCV(i, j, control_point);
            control_points.Append(control_point);
        }
    }



    //int cv_count_u = nurbs_surf.CVCount(0);  // U 方向的控制点数量
    //int cv_count_v = nurbs_surf.CVCount(1);  // V 方向的控制点数量
    //for (int i = 0; i < cv_count_u; ++i) {
    //    for (int j = 0; j < cv_count_v; ++j) {
        //  ON_3dPoint cv_point;
    //        nurbs_surf.GetCV(i, j, cv_point);
    //        std::cout << "CV[" << i << "][" << j << "] = (" << cv_point.x << ", " << cv_point.y << ", " << cv_point.z << ")" << std::endl;
    //    }
    //}

    // 构造一个包围盒
    ON_Point min_pt(0.0, 0.0, 0.0);
    ON_Point max_pt(1.0, 1.0, 1.0);
    ON_BoundingBox bbox(min_pt, max_pt);
    ON_BoundingBox bboxt(min_pt, max_pt);

    // 获取控制点数组的范围
    bboxt.Set(control_points);

    Eigen::MatrixXd ControlPnt_PX(n + 1, m + 1);
    Eigen::MatrixXd ControlPnt_PY(n + 1, m + 1);
    Eigen::MatrixXd ControlPnt_PZ(n + 1, m + 1);
    std::vector<Eigen::MatrixXd> Pw;

    ON_3dPoint cv1;
    for (int i = 0; i < nurbs_surf.CVCount(0); i++)
    {
        for (int j = 0; j < nurbs_surf.CVCount(1); j++)
        {

            nurbs_surf.GetCV(i, j, cv1);

            ControlPnt_PX(i, j) = cv1.x;
            ControlPnt_PY(i, j) = cv1.y;
            ControlPnt_PZ(i, j) = cv1.z;
        }
    }


    Pw.push_back(ControlPnt_PX);
    Pw.push_back(ControlPnt_PY);
    Pw.push_back(ControlPnt_PZ);

    vector<Eigen::MatrixXd> Pwt = TransformControlPoints(n, m, Pw, transform);
    /*  std::cout << "Pwt: " << "\n";
      for (int i = 0; i <= n; i++)
      {
          for (int j = 0; j <= m; j++)
          {
              std::cout << Pwt[0](i, j) << " " << Pwt[1](i, j) << " " << Pwt[2](i, j) << "   ";
          }
          std::cout << " " << "\n";
      }
      std::cout << "\n";*/
    CreatNURBS(p, q, n, m, Pwt, U, V, nurbs_surf1);
    //归一化曲面
    Vector3d transPoint = SurfaceNormalization(nurbs_surf, bbox, point);

    //归一化曲面
    Vector3d transPoint1 = SurfaceNormalizationt(nurbs_surf1, bbox, point);

    // 设置你想计算的u和v参数（比如曲面中点，u=0.5，v=0.5）
    double u = 0.5;
    double v = 0.5;

    // 确保曲面已成功创建
    if (nurbs_surf.IsValid()) {
        //std::cerr << "NURBS surface is valid!" << std::endl;
    }
    int dir = 1;
    // std::cout << "The Boundary1" << "\n";
    double point234[6];
    double point432[6];
    //// 记录开始时间点
    //auto start = std::chrono::high_resolution_clock::now();


    ///*******************************************************
    // 求取曲面上u, v参数对应的点
   // ON_3dPoint point1 = GetPointOnNURBSSurface(nurbs_surf, u, v);

    // 输出计算出的点的坐标
    /*std::cout << "Point at (u=" << u << ", v=" << v << "): "
        << point1.x << ", " << point1.y << ", " << point1.z << std::endl;*/
        /* ***********************************************************
          查找并输出 z = 0 的点*/
          //FindPointsWithZZero(nurbs_surf,transform);



    //// 记录开始时间点
    auto start = std::chrono::high_resolution_clock::now();

    //求曲面内的点 
    std::vector<std::vector<double>> para_uv;
    //int pqtest = ConvexHullOnSplinedSurface(nurbs_surf1);
    //int pqtest = NormalCone(nurbs_surf1);


    int pqtest = ConvexHullOnSurface(nurbs_surf1);
    /* std::cout << "曲面是否含有极点" << pqtest << "\n";
     std::cout << "The Surface:" << "\n";*/

     /*std::cout << "p = SplitTheSurface:" << p << "\n";*/

    vector<double> para_point;
    //小环存储向量
    std::vector<Point> loop;
    /*double u0 = 0.25;
    double v0 = 0.25;
    std::vector<double> tIntersect;
    bool testw = gradient_descent(u0, v0, 0.0005, 1000000, nurbs_surf1, tIntersect);
    if (testw) {
        for (int i = 0; i < tIntersect.size(); i++) {
            ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, tIntersect[0], tIntersect[1]);
            Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
            loop.push_back(point001);
        }
    }*/

    if (pqtest == 1) {


        int p = SplitTheSurface(nurbs_surf1, para_uv);
      
        //int delnumber = delthepoint(para_uv);
        int tengent = 0;
        for (int i = 0; i < para_uv.size(); i++) {
            ON_3dPoint original_point002 = GetPointOnNURBSSurface(nurbs_surf1, para_uv[i][0], para_uv[i][1]);
          
            if(abs(original_point002[2])<1e-4){
             ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, para_uv[i][0], para_uv[i][1]);
            Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
            loop.push_back(point001);
            para_uv.erase(para_uv.begin() + i);
            tengent = tengent + 1;
            }
        }

        //// 边界求交记录结束时间点
        //auto end = std::chrono::high_resolution_clock::now();

        //// 计算时间差并转换为毫秒
        //std::chrono::duration<double, std::milli> elapsed = end - start;

        //std::cout << "闭环检测时间: " << elapsed.count() << " 毫秒" << std::endl;


        // std::cout << "para_uv: " << "\n";

        for (int i = 0; i < para_uv.size(); i++) {
            /*for (int j = i + 1; j < para_uv.size(); j++) {
                double dis = abs(para_uv[i][0] - para_uv[j][0]) * abs(para_uv[i][0] - para_uv[j][0]) + abs(para_uv[i][1] - para_uv[j][1]) * abs(para_uv[i][1] - para_uv[j][1]);
                if (dis > 1e-3) {*/

                    //std::cout << para_uv[i][0] << " ,  " << para_uv[i][1];
                    ON_Curve* iso_curvepoint = nurbs_surf1.IsoCurve(0, para_uv[i][1]);  // 表示顶点处 u 方向等参线


                    ON_3dPoint point2 = GetPointOnNURBSSurface(nurbs_surf, para_uv[i][0], para_uv[i][1]);
                    // std::cout << "法向为（0,0,1）的点：" << point2[0] << " " << point2[1] << " " << point2[2] << "\n";
                    int k = para_uv.size();
                    Point point_ansuv = { point2[0] ,point2[1] ,point2[2] };
                    //  loop.push_back(point_ansuv);


                      /* std::cout << "法向为（0,0,1）的点对应等参线：" << "\n";*/


                    SplitTheinnerBoundary(iso_curvepoint, para_point);
                    /* for (int k = 0; k < para_point.size(); k++) {
                         ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, para_point[k], para_uv[k][1]);
                         Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
                         loop.push_back(point001);
                     }*/
                    //删除重复的parade_point
                    ////////////////////////////////////////////////
                    ///还没更改！！！！
                   /* for (int i = 0; i < para_point.size(); i++) {
                        ON_3dPoint original_point002 = GetPointOnNURBSSurface(nurbs_surf1, para_uv[i][0], para_uv[i][1]);

                        if (abs(original_point002[2]) < 1e-4) {
                            ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, para_uv[i][0], para_uv[i][1]);
                            Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
                            loop.push_back(point001);
                            para_uv.erase(para_uv.begin() + i);
                            
                        }
                    }*/
                   ///////////////////////////////
                   // ////////////////////////////
                     ////输出求得小环上的起始点
                     //std::cout << "para_point: " << "\n";
                     //for (int d = 0; d < para_point.size(); d++) {
                     //  /*  std::cout << para_point[d] << " ,  ";*/

                     //    ON_3dPoint point001 = GetPointOnNURBSSurface(nurbs_surf, para_point[d], para_uv[i][1]);
                     //   // Point point_ans001 = { point001[0] ,point001[1] ,point001[2] };
                     //    //loop.push_back(point_ans001);
                     //    std::cout <<"("<< point001[0] << " ,  " << point001[1] << " ,  " << point001[2]<<")";
                     //    std::cout << "\n";
                     //    //std::cout << "points on 法向为（0,0,1）的点对应等参线: " << "\n";
                     //}
                     




                    for (int k = 0; k < para_point.size(); k++) {
                        iso_curvepoint->Evaluate(para_point[k], 1, 3, point432);
                        if (abs(point432[2]) < 1e-4) {
                            /*std::cout << "(" << point432[0] << "," << point432[1] << "," << point432[2] << ")";
                            std::cout << "\n";*/
                            Point point_ans = { point432[0] ,point432[1] ,point432[2] };
                            if (whethertraced(viewoutput, point_ans) == 0) {
                                //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                                ON_3dPoint original_point432 = GetPointOnNURBSSurface(nurbs_surf, para_point[k], para_uv[i][1]);
                                Point original_point = { original_point432[0] ,original_point432[1] ,original_point432[2] };
                                /* cout << "逆变换后的点: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                                 std::cout << "trace出的点： " << "\n";*/
                                loop.push_back(original_point);
                                dir = 1;

                                vector<double> p1 = tracelineloop(nurbs_surf1, para_point[k], para_uv[i][1], dir);
                                double p11 = para_uv[i][1];
                                double p10 = para_point[k];
                                for (int j = 0; j < 1000 && p1[0] <= maxUknot && p1[1] <= maxVknot && p1[0] >= minUknot && p1[1] >= minVknot && whetherlooped(p1, p10, p11) == 0; j++) {//每一个点trace出来的点数

                                    ON_3dPoint point234 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
                                    // if (abs(point234[2]) < 1e-4) {
                                        // std::cout << "(" << point234[0] << "," << point234[1] << "," << point234[2] << ")";
                                   /* std::cout << "\n";*/
                                    Point point_ans = { point234[0] ,point234[1] ,point234[2] };

                                    /*  Point original_point = inverse_affine_transform(point_ans, affineMatrix);*/
                                      // cout << "trace的点逆变换后为: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                                     /* cout << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;*/
                                      //}
                                      //else {
                                        //  cout << "所求点不满足要求。" << endl;
                                      //}
                                    loop.push_back(point_ans);
                                    p1 = tracelineloop(nurbs_surf1, p1[0], p1[1], dir);
                                }
                                /* double dis = abs(p1[0] - original_point.x) + abs(p1[1] - original_point.y) + abs(p1[2] - original_point.z);
                                     if (dis < 1e-3) {
                                         loop.push_back(original_point);
                                     }*/
                                     // int Loopsize = loop.size();
                                viewoutput.push_back(loop);
                                loop.clear();

                                //dir = -1;
                                //loop.push_back(original_point);
                                //p1 = tracelineloop(nurbs_surf1, para_point[k], para_uv[i][0], dir);
                                //for (int j = 0; j < 1000 && p1[0] <= maxUknot && p1[1] <= maxVknot && p1[0] >= minUknot && p1[1] >= minVknot && whetherlooped(p1, p10, p11) == 0; j++) {//每一个点trace出来的点数
                                //    ON_3dPoint point234 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
                                //    // if (abs(point234[2]) < 1e-4) {
                                //        // std::cout << "(" << point234[0] << "," << point234[1] << "," << point234[2] << ")";
                                //   /* std::cout << "\n";*/
                                //    Point point_ans = { point234[0] ,point234[1] ,point234[2] };
                                //    if (whethertraced(viewoutput, point_ans) == 0) {

                                //        /*  Point original_point = inverse_affine_transform(point_ans, affineMatrix);*/
                                //          // cout << "trace的点逆变换后为: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                                //         /* cout << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;*/
                                //          //}
                                //          //else {
                                //            //  cout << "所求点不满足要求。" << endl;
                                //          //}
                                //        loop.push_back(point_ans);
                                //        p1 = tracelineloop(nurbs_surf1, p1[0], p1[1], dir);
                                //    }
                                //} 
                                /* dis = abs(p1[0] - original_point.x) + abs(p1[1] - original_point.y) + abs(p1[2] - original_point.z);
                                if (dis < 1e-3) {
                                    loop.push_back(original_point);
                                }*/
                                // int Loopsize = loop.size();
                                //viewoutput.push_back(loop);
                              //  loop.clear();

                            }
                        }
                    }

                }
            }
   
   
    
   
   
        //test area#########################################################################################
        // #################################################################################################
          // 提取一条等参线 (u 方向)
        std::vector<double> para_1, para_2, para_3, para_4;
        //double v_position = 0.0 * (nurbs_surf1.Domain(1).Min() + nurbs_surf1.Domain(1).Max());  // 
        ON_Curve* iso_curve1 = nurbs_surf1.IsoCurve(0, minUknot);  // 0 表示 u 方向等参线
        ON_Curve* iso_curve2 = nurbs_surf1.IsoCurve(0, maxUknot);
        ON_Curve* iso_curve3 = nurbs_surf1.IsoCurve(1, minVknot);
        ON_Curve* iso_curve4 = nurbs_surf1.IsoCurve(1, maxVknot);

        //输出的点列
        std::vector<Point> Boundary1;
        std::vector<Point> Boundary2;
        std::vector<Point> Boundary3;
        std::vector<Point> Boundary4;


        int values[] = { 1 ,-1};







        int ptest = ConvexHull(*iso_curve1);
        /* std::cout << "边界1是否有交点： " << ptest << "\n";
         std::cout << "para_1: " << "\n";*/
         // for (int i = 0; i < para_1.size(); i++)
             // std::cout << para_1[i] << " ,  ";
        /*  std::cout << "\n";
          std::cout << "points on Boundary1: " << "\n";*/
        if (ptest == 1) {

            SplitTheBoundary(iso_curve1, para_1);

            for (int i = 0; i < para_1.size(); i++) {
                iso_curve1->Evaluate(para_1[i], 1, 3, point234);
                if (abs(point234[2]) < 1e-4) {
                    /* std::cout << "" << point234[0] << " " << point234[1] << " " << point234[2] << "";
                     std::cout << "\n";*/
                    Point point_ans = { point234[0] ,point234[1] ,point234[2] };
                    if (whethertraced(viewoutput, point_ans) == 0) {
                        //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                        ON_3dPoint original_point432 = GetPointOnNURBSSurface(nurbs_surf, para_1[i], 0);
                        Point original_point = { original_point432[0] ,original_point432[1] ,original_point432[2] };
                        /* cout << "逆变换后的点: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                          std::cout << "trace出的点： " << "\n";*/
                        for (int dir : values) {
                            vector<double> p1 = traceline(nurbs_surf1, para_1[i], 0, dir);

                            Boundary1.push_back(original_point);
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
                            }
                            // int Bsize = Boundary1.size();
                            viewoutput.push_back(Boundary1);
                            Boundary1.clear();
                        }
                    }
                }
            }

        }



        ptest = ConvexHull(*iso_curve2);
        if (ptest == 1) {
            //std::cout << "The Boundary2" << "\n";
            SplitTheBoundary(iso_curve2, para_2);



            /*std::cout << "para_2: " << "\n";*/
           // for (int i = 0; i < para_2.size(); i++)
                /*  std::cout << para_2[i] << " ,  ";*/
              //std::cout << "\n";
             /* std::cout << "points on Boundary2: " << "\n";*/

            for (int i = 0; i < para_2.size(); i++) {
                iso_curve2->Evaluate(para_2[i], 1, 3, point234);
                if (abs(point234[2]) < 1e-4) {
                    /* std::cout << "" << point234[0] << " " << point234[1] << " " << point234[2] << "";
                     std::cout << "\n";*/
                    Point point_ans = { point234[0] ,point234[1] ,point234[2] };
                    if (whethertraced(viewoutput, point_ans) == 0) {
                        //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                        ON_3dPoint original_point432 = GetPointOnNURBSSurface(nurbs_surf, para_2[i], 1);
                        Point original_point = { original_point432[0] ,original_point432[1] ,original_point432[2] };
                        // cout << "逆变换后的点: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                          //std::cout << "trace出的点： " << "\n";
                        Boundary2.push_back(original_point);
                        for (int dir : values) {
                            vector<double> p1 = traceline(nurbs_surf1, para_2[i], 1, dir);

                            for (int j = 0; j < 1000 && p1[0] <= maxUknot && p1[1] <= maxVknot && p1[0] >= minUknot && p1[1] >= minVknot; j++) {//每一个点trace出来的点数

                                ON_3dPoint point432 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
                                // if (abs(point432[2]) < 1e-4) {
                                    // std::cout << "(" << point432[0] << "," << point432[1] << "," << point432[2] << ")";
                                //std::cout << "\n";
                                Point point_ans = { point432[0] ,point432[1] ,point432[2] };

                                Boundary2.push_back(point_ans);

                                //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                                // cout << "trace的点逆变换后为: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                                //cout << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                              //***** cout <<  point_ans.x << " " << point_ans.y << " " << point_ans.z << " " << endl;
                                //}
                                //else {
                                  //  cout << "所求点不满足要求。" << endl;
                                //}
                                p1 = traceline(nurbs_surf1, p1[0], p1[1], dir);
                            }
                            viewoutput.push_back(Boundary2);
                            Boundary2.clear();
                        }
                    }
                }
            }

        }



        ptest = ConvexHull(*iso_curve3);
        if (ptest == 1) {
            /* std::cout << "The Boundary3" << "\n";*/
            SplitTheBoundary(iso_curve3, para_3);


            /* std::cout << "para_3: " << "\n";*/
             //for (int i = 0; i < para_3.size(); i++)
             //   /* std::cout << para_3[i] << " ,  ";
             //std::cout << "\n";*/
            /* std::cout << "points on Boundary3: " << "\n";*/



            for (int i = 0; i < para_3.size(); i++) {
                iso_curve3->Evaluate(para_3[i], 1, 3, point432);
                if (abs(point432[2]) < 1e-4) {
                    /* std::cout << "" << point432[0] << " " << point432[1] << " " << point432[2] << "";
                     std::cout << "\n";*/
                    Point point_ans = { point432[0] ,point432[1] ,point432[2] };
                    if (whethertraced(viewoutput, point_ans) == 0) {
                        //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                        ON_3dPoint original_point432 = GetPointOnNURBSSurface(nurbs_surf, 0, para_3[i]);
                        Point original_point = { original_point432[0] ,original_point432[1] ,original_point432[2] };
                        /*cout << "逆变换后的点: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;*/
                         //std::cout << "trace出的点： " << "\n";
                        Boundary3.push_back(original_point);
                        for (int dir : values) {
                            vector<double> p1 = traceline(nurbs_surf1, 0, para_3[i], dir);

                            for (int j = 0; j < 1000 && p1[0] <= maxUknot && p1[1] <= maxVknot && p1[0] >= minUknot && p1[1] >= minVknot; j++) {//每一个点trace出来的点数

                                ON_3dPoint point234 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
                                // if (abs(point234[2]) < 1e-4) {
                                    // std::cout << "(" << point234[0] << "," << point234[1] << "," << point234[2] << ")";
                                //std::cout << "\n";
                                Point point_ans = { point234[0] ,point234[1] ,point234[2] };

                                Boundary3.push_back(point_ans);
                                // Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                                 // cout << "trace的点逆变换后为: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                                 //cout << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                                  // cout <<  point_ans.x << " " << point_ans.y << " " << point_ans.z << " " << endl;
                                 //}
                                 //else {
                                   //  cout << "所求点不满足要求。" << endl;
                                 //}
                                p1 = traceline(nurbs_surf1, p1[0], p1[1], dir);
                            }
                            viewoutput.push_back(Boundary3);
                            Boundary3.clear();
                        }
                    }
                }
            }

        }

        ptest = ConvexHull(*iso_curve4);
        if (ptest == 1) {

            /* std::cout << "The Boundary4" << "\n";*/
            SplitTheBoundary(iso_curve4, para_4);



            /*std::cout << "para_4: " << "\n";*/
            //for (int i = 0; i < para_4.size(); i++)
                // std::cout << para_4[i] << " ,  ";
            /* std::cout << "\n";
             std::cout << "points on Boundary4: " << "\n";*/



            for (int i = 0; i < para_4.size(); i++) {
                iso_curve4->Evaluate(para_4[i], 1, 3, point432);
                if (abs(point432[2]) < 1e-4) {
                    /* std::cout << "" << point432[0] << " " << point432[1] << " " << point432[2] << "";
                     std::cout << "\n";*/
                    Point point_ans = { point432[0] ,point432[1] ,point432[2] };
                    if (whethertraced(viewoutput, point_ans) == 0) {
                        //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                        ON_3dPoint original_point432 = GetPointOnNURBSSurface(nurbs_surf, 1, para_4[i]);
                        Point original_point = { original_point432[0] ,original_point432[1] ,original_point432[2] };
                        /* cout << "逆变换后的点: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                         std::cout << "trace出的点： " << "\n";*/
                        Boundary4.push_back(original_point);
                        for (int dir : values) {
                            vector<double> p1 = traceline(nurbs_surf1, 1, para_4[i], dir);

                            for (int j = 0; j < 1000 && p1[0] <= maxUknot && p1[1] <= maxVknot && p1[0] >= minUknot && p1[1] >= minVknot; j++) {//每一个点trace出来的点数
                                double testp = p1[0];
                                double testp1 = p1[1];
                                ON_3dPoint point234 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
                                // if (abs(point234[2]) < 1e-4) {
                                    // std::cout << "(" << point234[0] << "," << point234[1] << "," << point234[2] << ")";
                                //std::cout << "\n";
                                Point point_ans = { point234[0] ,point234[1] ,point234[2] };
                                Boundary4.push_back(point_ans);
                                //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                              //********cout << point_ans.x << " " << point_ans.y << " " << point_ans.z << " " << endl;
                                // cout << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                                  //}
                                  //else {
                                    //  cout << "所求点不满足要求。" << endl;
                                  //}
                                p1 = traceline(nurbs_surf1, p1[0], p1[1], dir);
                            }
                            viewoutput.push_back(Boundary4);
                            Boundary4.clear();
                        }




                    }
                }
            }

        }



        ////边界求交记录结束时间点
         //auto end = std::chrono::high_resolution_clock::now();

         //// 计算时间差并转换为毫秒
         //std::chrono::duration<double, std::milli> elapsed = end - start;

         //std::cout << "边界求交检测时间: " << elapsed.count() << " 毫秒" << std::endl;



                //将所有交点合并起来


               /* viewoutput.insert(viewoutput.end(), Boundary1.begin(), Boundary1.end());
                viewoutput.insert(viewoutput.end(), Boundary2.begin(), Boundary2.end());
                viewoutput.insert(viewoutput.end(), Boundary3.begin(), Boundary3.end());
                viewoutput.insert(viewoutput.end(), Boundary4.begin(), Boundary4.end());
                viewoutput.insert(viewoutput.end(), loop.begin(), loop.end());*/



                //std::cout << "\n";
                //std::cout << "points on Surface: " << "\n";


/*cout << "逆变换后的点: \n";*/
// int psize = para_uv.size();
 /*  cout << "顶点个数:" << psize<<"\n";*/

 //for (int i = 0; i < para_uv.size(); i++) {
 //    ON_3dPoint point1 = GetPointOnNURBSSurface(nurbs_surf, para_uv[i][0], para_uv[i][1]);
 //    double p0 = para_uv[i][0];
 //    double p1 = para_uv[i][1];
 //    // 输出计算出的点的坐标
 //    //***std::cout <<  point1.x << " " << point1.y << " " << point1.z << std::endl;

 //   /* Point point_ans = { point1.x ,point1.y ,point1.z };
 //    Point original_point = inverse_affine_transform(point_ans, affineMatrix);
 //    cout << original_point.x << " " << original_point.y << " " << original_point.z << " \n" << endl;*/

 //}


//if (pqtest != 1) {
//    // 边界求交记录结束时间点
//    auto end = std::chrono::high_resolution_clock::now();

//    // 计算时间差并转换为毫秒
//    std::chrono::duration<double, std::milli> elapsed = end - start;

//    std::cout << "闭环检测时间: " << elapsed.count() << " 毫秒" << std::endl;
//}

        int i;
        int j;
        Point pointOut;



        //还原
        double xmin = bboxt.m_min.x;
        double ymin = bboxt.m_min.y;
        double zmin = bboxt.m_min.z;
        double xmax = bboxt.m_max.x;
        double ymax = bboxt.m_max.y;
        double zmax = bboxt.m_max.z;


        double cube_size = 1.0;
        double max_size = (std::max)(xmax - xmin, ymax - ymin);
        max_size = (std::max)(max_size, zmax - zmin);

        double a = transPoint[0];
        double b = transPoint[1];
        double c = transPoint[2];
        for (i = 0; i < viewoutput.size(); i++) {
            for (j = 0; j < viewoutput[i].size(); j++) {
                pointOut = viewoutput[i][j];
                pointOut.x = (pointOut.x + transPoint[0]) * max_size / cube_size + xmin;
                pointOut.y = (pointOut.y + transPoint[1]) * max_size / cube_size + ymin;
                pointOut.z = (pointOut.z + transPoint[2]) * max_size / cube_size + zmin;
                viewoutput[i][j] = pointOut;
            }
        }
        auto end = std::chrono::high_resolution_clock::now();

        // 计算时间差并转换为毫秒
        std::chrono::duration<double, std::milli> elapsed = end - start;

        std::cout << "求交总时间: " << elapsed.count() << " 毫秒" << std::endl;
        //////////////////////////////////////////////////////////////////////////////////////////////////////////


        std::cout << "para_size: " << para_1.size() << ", " << para_2.size() << ", " << para_3.size() << ", " << para_4.size() << "\n";
        std::cout << "para_point: " << para_point.size() << "\n"<<"para_uv(0,0,1)" << para_uv.size();

    
   //     // 遍历每一层，查看该层的交点个数（分支数）
   //for (size_t i = 0; i < viewoutput.size(); i++) {
   //    size_t num_branches = viewoutput[i].size();
   //    std::cout << "第 " << i << " 层的分支个数: " << num_branches << std::endl;

   //    // 如果需要，还可以查看每个分支的具体坐标
   //    for (size_t j = 0; j < viewoutput[i].size(); j++) {
   //        std::cout << "  分支 " << j << ": " << viewoutput[i][j].x << viewoutput[i][j].y << viewoutput[i][j].z << std::endl;
   //    }
   //}
    return viewoutput;

    //将点坐标输出到txt文件中
    //std::string name = "D:\\安装文件\\project\\example\\intersectionspline.txt"; // Windows
    //std::ofstream outfile(name);
    ////将Boundary1234和loop输出到文件
    //int i;
    //Point pointOut;



    ////还原
    //double xmin = bboxt.m_min.x;
    //double ymin = bboxt.m_min.y;
    //double zmin = bboxt.m_min.z;
    //double xmax = bboxt.m_max.x;
    //double ymax = bboxt.m_max.y;
    //double zmax = bboxt.m_max.z;
    //double cube_size = 1.0;
    //double max_size = (std::max)(xmax - xmin, ymax - ymin);
    //max_size = (std::max)(max_size, zmax - zmin);

    //double a = transPoint[0];
    //double b = transPoint[1];
    //double c = transPoint[2];
    //for (i = 0; i < Boundary1.size(); i++) {
    //    pointOut = Boundary1[i];
    //    pointOut.x = (pointOut.x + transPoint[0]) * max_size / cube_size + xmin;
    //    pointOut.y = (pointOut.y + transPoint[1]) * max_size / cube_size + ymin;
    //    pointOut.z = (pointOut.z + transPoint[2]) * max_size / cube_size + zmin;
    //    outfile << pointOut.x << " " << pointOut.y << " " << pointOut.z << "\n";
    //}
    //for (i = 0; i < Boundary2.size(); i++) {
    //    pointOut = Boundary2[i];
    //    pointOut.x = (pointOut.x + transPoint[0]) * max_size / cube_size + xmin;
    //    pointOut.y = (pointOut.y + transPoint[1]) * max_size / cube_size + ymin;
    //    pointOut.z = (pointOut.z + transPoint[2]) * max_size / cube_size + zmin;
    //    outfile << pointOut.x << " " << pointOut.y << " " << pointOut.z << "\n";
    //}
    //for (i = 0; i < Boundary3.size(); i++) {
    //    pointOut = Boundary3[i];
    //    pointOut.x = (pointOut.x + transPoint[0]) * max_size / cube_size + xmin;
    //    pointOut.y = (pointOut.y + transPoint[1]) * max_size / cube_size + ymin;
    //    pointOut.z = (pointOut.z + transPoint[2]) * max_size / cube_size + zmin;
    //    outfile << pointOut.x << " " << pointOut.y << " " << pointOut.z << "\n";
    //}

    //for (i = 0; i < Boundary4.size(); i++) {
    //    pointOut = Boundary4[i];
    //    pointOut.x = (pointOut.x + transPoint[0]) * max_size / cube_size + xmin;
    //    pointOut.y = (pointOut.y + transPoint[1]) * max_size / cube_size + ymin;
    //    pointOut.z = (pointOut.z + transPoint[2]) * max_size / cube_size + zmin;
    //    outfile << pointOut.x << " " << pointOut.y << " " << pointOut.z << "\n";
    //}

    //for (i = 0; i < loop.size(); i++) {
    //    pointOut = loop[i];
    //    pointOut.x = (pointOut.x + transPoint[0]) * max_size / cube_size + xmin;
    //    pointOut.y = (pointOut.y + transPoint[1]) * max_size / cube_size + ymin;
    //    pointOut.z = (pointOut.z + transPoint[2]) * max_size / cube_size + zmin;
    //    outfile << pointOut.x << " " << pointOut.y << " " << pointOut.z << "\n";
    //}



    //outfile.close();
    ////  

 //// 记录开始时间点
 //auto start = std::chrono::high_resolution_clock::now();

     /*  double u0 = 0.25;
       double v0 = 0.25;
       std::vector<double> tIntersect;
    bool testw = gradient_descent(u0, v0, 0.0005, 1000000, nurbs_surf1, tIntersect);
    if (testw) {*/
 //       
 //         //边界求交记录结束时间点
 //auto end = std::chrono::high_resolution_clock::now();

 // //计算时间差并转换为毫秒
 //std::chrono::duration<double, std::milli> elapsed = end - start;

 //std::cout << "闭环检测时间: " << elapsed.count() << " 毫秒" << std::endl;

 //      if (testw) {
 //          ON_3dPoint point1 = GetPointOnNURBSSurface(nurbs_surf1, tIntersect[0], tIntersect[1]);
 //          ON_3dPoint point2 = GetPointOnNURBSSurface(nurbs_surf, tIntersect[0], tIntersect[1]);
 //          std::cout << "法向为（0,0,1）的点：" << point2[0] << " " << point2[1] << " " << point2[2] << "\n";
 //          ON_Curve* iso_curvepoint = nurbs_surf1.IsoCurve(0, tIntersect[1]);  // 表示顶点处 u 方向等参线
 //          vector<double> para_point;


 //         
 //          std::cout << "法向为（0,0,1）的点对应等参线：" << "\n";
 //          SplitTheBoundary(iso_curvepoint, para_point);
 //          std::cout << "para_point: " << "\n";
 //          for (int i = 0; i < para_point.size(); i++)
 //              std::cout << para_point[i] << " ,  ";
 //          std::cout << "\n";
 //          std::cout << "points on 法向为（0,0,1）的点对应等参线: " << "\n";
 //          for (int i = 0; i < para_point.size(); i++) {
 //              iso_curvepoint->Evaluate(para_point[i], 1, 3, point432);
 //              if (abs(point432[2]) < 1e-4) {
 //                  std::cout << "(" << point432[0] << "," << point432[1] << "," << point432[2] << ")";
 //                  std::cout << "\n";
 //                  Point point_ans = { point432[0] ,point432[1] ,point432[2] };
 //                  Point original_point = inverse_affine_transform(point_ans, affineMatrix);
 //                  std::cout << "逆变换后的点: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
 //                  std::cout << "trace出的点： " << "\n";
 //                  int dirs[2] = { 1,-1 };
 //                  for (int k = 0; k < 2; k++) {
 //                      vector<double> p1 = traceline(nurbs_surf1, para_point[i], tIntersect[1], dirs[k]);

 //                      for (int j = 0; j < 50 && p1[0] <= 1 && p1[1] <= 1 && p1[0] >= 0 && p1[1] >= 0; j++) {//每一个点trace出来的点数

 //                          ON_3dPoint point234 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
 //                          double p10 = p1[0];
 //                          double p11 = p1[1];
 //                          // if (abs(point234[2]) < 1e-4) {
 //                              // std::cout << "(" << point234[0] << "," << point234[1] << "," << point234[2] << ")";
 //                          std::cout << "\n";
 //                          Point point_ans = { point234[0] ,point234[1] ,point234[2] };
 //                          //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
 //                           cout << point_ans.x << " " << point_ans.y << " " << point_ans.z << " " << endl;
 //                          //std::cout << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
 //                          //}
 //                          //else {
 //                            //  cout << "所求点不满足要求。" << endl;
 //                          //}
 //                          p1 = traceline(nurbs_surf1, p1[0], p1[1], dirs[k]);
 //                      }
 //                  }
 //              }
 //              //    //输出计算出的点的坐标
 //              std::cout << "Iteration at (u=" << tIntersect[0] << ", v=" << tIntersect[1] << "): "
 //                  << point1.x << ", " << point1.y << ", " << point1.z << std::endl;
 //              Point point_ans = { point1.x ,point1.y ,point1.z };
 //              Point original_point = inverse_affine_transform(point_ans, affineMatrix);
 //              cout << "逆变换后的点: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;

 //              }
 //         }

       //导出文件创建并打开输出文件
      // std::string name = "SplineSurface.nurbs";当前运行文件
       //std::string name = "D:\\安装文件\\project\\example\\B-spline_Boat.nurbs"; // Windows
       //std::ofstream outfile(name);
       //////写入 NURBS 曲面的基本信息
       //outfile << "process face 0 " << "\n";
       //outfile << "Bspline Surf U degree " << p << " V degree " << q << " U Knots " << nurbs_surf1.CVCount(0) << " V Knots " << nurbs_surf1.CVCount(1) << "\n";
       //ON_3dPoint Point;
       //int numCtrlPointsU = nurbs_surf1.CVCount(0);  // U 方向的控制点数量
       //int numCtrlPointsV = nurbs_surf1.CVCount(1);  // V 方向的控制点数量
       //for (int i = 0; i < numCtrlPointsU; i++) {

       //    outfile << "{";
       //    for (int j = 0; j < numCtrlPointsV - 1; j++) {
       //        nurbs_surf1.GetCV(i, j, Point);
       //        outfile << "{" << Point.x << "," << Point.y << "," << Point.z << " }";
       //        outfile << ",";
       //    }
       //    nurbs_surf1.GetCV(i, numCtrlPointsV - 1, Point);
       //    outfile << "{" << Point.x << "," << Point.y << "," << Point.z << " }";
       //    outfile << "},\n";
       //}
       //outfile << "Knot Sequence: " << "\n";
       //outfile << "{" << nurbs_surf1.Knot(0, 0) << ",";
       //for (int i = 0; i < nurbs_surf1.KnotCount(0); i++)
       //    outfile << nurbs_surf1.Knot(0, i) << ", ";
       //outfile << nurbs_surf1.Knot(0, nurbs_surf1.KnotCount(0) - 1) << "},\n ";

       //outfile << "{" << nurbs_surf1.Knot(1, 0) << ",";
       //for (int i = 0; i < nurbs_surf1.KnotCount(1); i++)
       //    outfile << nurbs_surf1.Knot(1, i) << ",";
       //outfile << nurbs_surf1.Knot(1, nurbs_surf1.KnotCount(1) - 1) << "}\n ";

       //outfile.close();


       //输出所求交点文件
       //std::string name = "D:\\安装文件\\project\\example\\intersection.nurbs1"; // Windows
       //std::ofstream outfile(name);


       //outfile.close();



          // int pcone = NormalCone(nurbs_surf1);
          // std::cout << "曲面是否含有极点cone" << pqtest << "\n";



       //    //////###############################################################################################
       //    //     求导部分示例
       //       // // 设置参数 u，在该点计算曲线导数
       //       // double t123 = 0.5;
       //       // std::cout << " t = " << t123;
       //       // // 存储曲线在 u 处的点和导数
       //       // ON_3dPoint point123(0.0,0.0,0.0);                 // 曲线在参数 u 处的点
       //       //// ON_3dVector derivatives[2];       // 一阶导数 (切线)，存储在 [0] 位置，二阶导数 (加速度) 在 [1] 位置
       //       // double derivatives[6];
       //       // ON_NurbsCurve* nurbs = ON_NurbsCurve::Cast(iso_curve1);
       //       // 
       //       // // Evaluate 函数参数: u 值, 求导阶数, 返回数组大小(3表示返回点及其导数)
       //       // bool success = nurbs->Evaluate(t123, 1,3,derivatives);

       //       // if (success)
       //       // {
       //       //     // 输出曲线在参数 u 处的点
       //       //     std::cout << "Point at t = " << t123 <<  ": ("
       //       //         << derivatives[0] << ", " << derivatives[1] << ", " << derivatives[2] << ")" << std::endl;

       //       //     // 输出一阶导数（切线方向）
       //       //     std::cout << "First derivative (tangent) at t = " << t123 << "; (" <<derivatives[3] <<derivatives[4] << derivatives[5] << ")" << std::endl;
       //       // }
       //       // else
       //       // {
       //       //     std::cout << "Failed to evaluate NURBS curve at t = " << t123 << std::endl;
       //       // }
       //    //####################################################################################################################


       //        //打印仿射变换后的控制点
       //       /* std::cout << "Pwt: " << "\n";
       //        for (int i = 0; i <= n; i++)
       //        {
       //            for (int j = 0; j <= m; j++)
       //            {
       //                std::cout << Pwt[0](i, j) << " " << Pwt[1](i, j) << " " << Pwt[2](i, j) << "   ";
       //            }
       //            std::cout << " " << "\n";
       //        }
       //        std::cout << "\n";*/

       //        // ###########################################################################################################
       //         //###############################################################################################################
       //    ON::End(); // 清理OpenNURBS库





       //    TotalTime_end = clock();

       //    std::cout << "Int_Pnts.size(): " << Int_Pnts.size() / 3 << "\n";


       //  

       //    std::string name;
       //    name = "Intersection_Eg1.txt";

       //    std::ofstream ofs;

       //    ofs.open(name, std::ios::out);
       //    for (int i = 0; i < Int_Pnts.size() / 3; i++)
       //    {
       //        ofs << Int_Pnts[3 * i] << " " << Int_Pnts[3 * i + 1] << " " << Int_Pnts[3 * i + 2] << "\n";
       //    }
       //    ofs.close();


       //    std::cout << "The total computing time = " << (double)(TotalTime_end - TotalTime_begin) / CLOCKS_PER_SEC << "\n" << "\n";


       //}
       //// 记录结束时间点
       //auto end = std::chrono::high_resolution_clock::now();

       //// 计算时间差并转换为毫秒
       //std::chrono::duration<double, std::milli> elapsed = end - start;

       //std::cout << "执行时间: " << elapsed.count() << " 毫秒" << std::endl;


}

void nurbsintersection_OCC(ON_NurbsSurface& nurbs1, ON_NurbsSurface& nurbs2, std::vector<std::vector<double>>& intersection_points)
{
    
    int num_u = nurbs1.CVCount(0), num_v = nurbs1.CVCount(1);

    TColgp_Array2OfPnt points1(1, num_u, 1, num_v);
    for (int i = 1; i <= num_u; i++)
    {
        for (int j = 1; j <= num_v; j++)
        {
            ON_3dPoint CtrPoint;
            nurbs1.GetCV(i - 1, j - 1, CtrPoint);
            points1.SetValue(i, j, gp_Pnt(CtrPoint.x, CtrPoint.y, CtrPoint.z));
        }
    }

    num_u = nurbs2.CVCount(0);
    num_v = nurbs2.CVCount(1);

    TColgp_Array2OfPnt points2(1, num_u, 1, num_v);
    for (int i = 1; i <= num_u; i++)
    {
        for (int j = 1; j <= num_v; j++)
        {
            ON_3dPoint CtrPoint;
            nurbs2.GetCV(i - 1, j - 1, CtrPoint);
            points2.SetValue(i, j, gp_Pnt(CtrPoint.x, CtrPoint.y, CtrPoint.z));
        }
    }

    // 获取数组的下标范围打印控制点////////////////////////////////////////////////////////////////////////
    Standard_Integer lowerRow = points2.LowerRow();
    Standard_Integer upperRow = points2.UpperRow();
    Standard_Integer lowerCol = points2.LowerCol();
    Standard_Integer upperCol = points2.UpperCol();

    //std::cout << "平面控制点" << (upperRow - lowerRow + 1)
    //    << " 行, " << (upperCol - lowerCol + 1) << " 列):" << std::endl;

    //for (Standard_Integer r = lowerRow; r <= upperRow; ++r) {
    //    for (Standard_Integer c = lowerCol; c <= upperCol; ++c) {
    //        const gp_Pnt& p = points2(r, c);
    //        std::cout  << "(" << p.X() << ", " << p.Y() << ", " << p.Z() << ")";

    //        // 每行结束后换行
    //        if (c < upperCol) {
    //            std::cout << "\t";
    //        }
    //    }
    //    std::cout << std::endl;
    //}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    int num1 = 1;
    for (int i = 1; i < nurbs1.m_knot_capacity[0]; i++)
    {
        if (nurbs1.m_knot[0][i] > nurbs1.m_knot[0][i - 1]) num1++;
    }
    TColStd_Array1OfReal uKnots1(1, num1);
    TColStd_Array1OfInteger uMults1(1, num1);
    uKnots1.SetValue(1, nurbs1.m_knot[0][0]);
    int mult = 2;
    int k = 1;
    for (int i = 1; i < nurbs1.m_knot_capacity[0]; i++)
    {
        if (nurbs1.m_knot[0][i] > nurbs1.m_knot[0][i - 1])
        {
            uKnots1.SetValue(k + 1, nurbs1.m_knot[0][i]);
            uMults1.SetValue(k, mult);
            mult = 1;
            k++;
        }
        else
        {
            mult++;
        }
    }
    uMults1.SetValue(k, mult + 1);

    int num2 = 1;
    for (int i = 1; i < nurbs1.m_knot_capacity[1]; i++)
    {
        if (nurbs1.m_knot[1][i] > nurbs1.m_knot[1][i - 1]) num2++;
    }
    TColStd_Array1OfReal vKnots1(1, num2);
    TColStd_Array1OfInteger vMults1(1, num2);
    vKnots1.SetValue(1, nurbs1.m_knot[1][0]);
    mult = 2;
    k = 1;
    for (int i = 1; i < nurbs1.m_knot_capacity[1]; i++)
    {
        if (nurbs1.m_knot[1][i] > nurbs1.m_knot[1][i - 1])
        {
            vKnots1.SetValue(k + 1, nurbs1.m_knot[1][i]);
            vMults1.SetValue(k, mult);
            mult = 1;
            k++;
        }
        else
        {
            mult++;
        }
    }
    vMults1.SetValue(k, mult + 1);

    num1 = 1;
    for (int i = 1; i < nurbs2.m_knot_capacity[0]; i++)
    {
        if (nurbs2.m_knot[0][i] > nurbs2.m_knot[0][i - 1]) num1++;
    }
    TColStd_Array1OfReal uKnots2(1, num1);
    TColStd_Array1OfInteger uMults2(1, num1);
    uKnots2.SetValue(1, nurbs2.m_knot[0][0]);
    mult = 2;
    k = 1;
    for (int i = 1; i < nurbs2.m_knot_capacity[0]; i++)
    {
        if (nurbs2.m_knot[0][i] > nurbs2.m_knot[0][i - 1])
        {
            uKnots2.SetValue(k + 1, nurbs2.m_knot[0][i]);
            uMults2.SetValue(k, mult);
            mult = 1;
            k++;
        }
        else
        {
            mult++;
        }
    }
    uMults2.SetValue(k, mult + 1);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*  Standard_Integer lower = uKnots2.Lower();
    Standard_Integer upper = uKnots2.Upper();

    std::cout << "U方向节点向量 (索引 " << lower << " 到 " << upper << "):" << std::endl;

    for (Standard_Integer i = lower; i <= upper; ++i) {
        std::cout << "uKnots[" << i << "] = " << uKnots2(i) << std::endl;
    }

    lower = uMults2.Lower();
    upper = uMults2.Upper();

    std::cout << "U方向节点重复度数组 (索引 " << lower << " 到 " << upper << "):" << std::endl;

    for (Standard_Integer i = lower; i <= upper; ++i) {
        std::cout << "uMults2[" << i << "] = " << uMults2(i) << std::endl;
    }*/
    ////////////////////////////////////////////////////////

    num2 = 1;
    for (int i = 1; i < nurbs2.m_knot_capacity[1]; i++)
    {
        if (nurbs2.m_knot[1][i] > nurbs2.m_knot[1][i - 1]) num2++;
    }
    TColStd_Array1OfReal vKnots2(1, num2);
    TColStd_Array1OfInteger vMults2(1, num2);
    vKnots2.SetValue(1, nurbs2.m_knot[1][0]);
    mult = 2;
    k = 1;
    for (int i = 1; i < nurbs2.m_knot_capacity[1]; i++)
    {
        if (nurbs2.m_knot[1][i] > nurbs2.m_knot[1][i - 1])
        {
            vKnots2.SetValue(k + 1, nurbs2.m_knot[1][i]);
            vMults2.SetValue(k, mult);
            mult = 1;
            k++;
        }
        else
        {
            mult++;
        }
    }
    vMults2.SetValue(k, mult + 1);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*  lower = vKnots2.Lower();
    upper = vKnots2.Upper();

    std::cout << "V方向节点向量 (索引 " << lower << " 到 " << upper << "):" << std::endl;

    for (Standard_Integer i = lower; i <= upper; ++i) {
        std::cout << "vKnots[" << i << "] = " << vKnots2(i) << std::endl;
    }
    lower = vMults2.Lower();
    upper = vMults2.Upper();

    std::cout << "V方向节点重复度数组 (索引 " << lower << " 到 " << upper << "):" << std::endl;

    for (Standard_Integer i = lower; i <= upper; ++i) {
        std::cout << "vMults2[" << i << "] = " << vMults2(i) << std::endl;
    }*/
    ////////////////////////////////////////////////////////

    int degreeu1 = nurbs1.Degree(0);
    int degreev1 = nurbs1.Degree(1);

    int degreeu2 = nurbs2.Degree(0);
    int degreev2 = nurbs2.Degree(1);

    Geom_BSplineSurface surf1(points1, uKnots1, vKnots1, uMults1, vMults1, degreeu1, degreev1);
    Geom_BSplineSurface surf2(points2, uKnots2, vKnots2, uMults2, vMults2, degreeu2, degreev2);



    Handle(Geom_Surface) h1 = new Geom_BSplineSurface(surf1);
    Handle(Geom_Surface) h2 = new Geom_BSplineSurface(surf2);

    GeomAdaptor_Surface adaptorSurface1(h1);
    Handle(Adaptor3d_Surface)surface1 = new GeomAdaptor_Surface(adaptorSurface1);
    Handle(Adaptor3d_TopolTool)tool1 = new Adaptor3d_TopolTool(surface1);

    GeomAdaptor_Surface adaptorSurface2(h2);
    Handle(Adaptor3d_Surface)surface2 = new GeomAdaptor_Surface(adaptorSurface2);
    Handle(Adaptor3d_TopolTool)tool2 = new Adaptor3d_TopolTool(surface2);

    Standard_Real tolArc = 1.0e-9;
    Standard_Real tolTang = 1.0e-6;

    IntPatch_Intersection t = IntPatch_Intersection();
    auto begin2 = clock();
    //// 记录开始时间点
    auto start = std::chrono::high_resolution_clock::now();
    t.Perform(surface1, tool1, surface2, tool2, tolArc, tolTang);
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差并转换为毫秒
    std::chrono::duration<double, std::milli> elapsed = end - start;

    std::cout << "occ求交总时间: " << elapsed.count() << " 毫秒" << std::endl;
    auto end2 = clock();
    std::cout << "intersection occ success! Time: " << (double)(end2 - begin2) / CLOCKS_PER_SEC << std::endl;
    double tracing_time_OCC = (double)(end2 - begin2) / CLOCKS_PER_SEC;

   
   
    //std::cout << "is tangent: " << (bool)t.TangentFaces() << std::endl;
    for (int i = 1; i <= t.NbLines(); i++)
    {
        Handle(IntPatch_Line) curve1 = t.Line(i);
        Handle(IntPatch_WLine) curve = Handle(IntPatch_WLine)::DownCast(curve1);
        int num = curve->NbPnts();
        std::vector<double> tmp_curve;
        tmp_curve.clear();
        for (int j = 1; j <= num; j++)
        {
            IntSurf_PntOn2S point = curve->Point(j);
            tmp_curve.push_back(point.Value().X());
            tmp_curve.push_back(point.Value().Y());
            tmp_curve.push_back(point.Value().Z());
        }
        intersection_points.push_back(tmp_curve);
        tmp_curve.clear();
    }
    // 遍历每一层，查看该层的交点个数（分支数）
    //for (size_t i = 0; i < intersection_points.size(); i++) {
    //    size_t num_branches = intersection_points[i].size();
    //    std::cout << "第 " << i << " 层的分支个数: " << num_branches << std::endl;

    //    // 如果需要，还可以查看每个分支的具体坐标
    //    for (size_t j = 0; j < intersection_points[i].size(); j++) {
    //        std::cout << "  分支 " << j << ": " << intersection_points[i][j] << std::endl;
    //    }
    //}
   
}



void nurbsintersection_Parasolid( ON_NurbsSurface& nurbs1, ON_NurbsSurface& nurbs2, std::vector<std::vector<double>>& intersection_points)
{
  
    // 1. 转换为 Parasolid 曲面
    PK_BSURF_t bsurf1 = PK_ENTITY_null;
    PK_BSURF_t bsurf2 = PK_ENTITY_null;

    PK_ERROR_code_t err1 = create_bsurf_from_onnurbs(nurbs1, bsurf1);
    PK_ERROR_code_t err2 = create_bsurf_from_onnurbs(nurbs2, bsurf2);
    if (err1 != PK_ERROR_no_errors || bsurf1 == PK_ENTITY_null) {
        std::cerr << "Parasolid conversion failed for surface 1: " << err1 << std::endl;
        if (bsurf2 != PK_ENTITY_null)
            PK_ENTITY_delete(1, &bsurf2);
        return;
    }
    if (err2 != PK_ERROR_no_errors || bsurf2 == PK_ENTITY_null) {
        std::cerr << "Parasolid conversion failed for surface 2: " << err2 << std::endl;
        PK_ENTITY_delete(1, &bsurf1);
        return;
    }

    // 2. 求交
    PK_SURF_intersect_surf_o_t opts;
    PK_SURF_intersect_surf_o_m(opts);

    int n_vectors = 0, n_curves = 0;
    PK_VECTOR_t* vectors = nullptr;
    PK_CURVE_t* curves = nullptr;
    PK_INTERVAL_t* bounds = nullptr;
    PK_intersect_curve_t* types = nullptr;

    PK_ERROR_code_t err = PK_SURF_intersect_surf(
        bsurf1, bsurf2, &opts,
        &n_vectors, &vectors,
        &n_curves, &curves, &bounds, &types
    );
    if (err != PK_ERROR_no_errors) {
        std::cerr << "Parasolid surface intersection failed: " << err << std::endl;
        if (vectors) PK_MEMORY_free(vectors);
        if (curves)  PK_MEMORY_free(curves);
        if (bounds)  PK_MEMORY_free(bounds);
        if (types)   PK_MEMORY_free(types);
        PK_ENTITY_delete(1, &bsurf1);
        PK_ENTITY_delete(1, &bsurf2);
        return;
    }

 

    // 3. 对每条交线采样，输出点列（格式与 OCC 版本一致：每层是 x,y,z,x,y,z,...）
    const int samples_per_curve = 100;
    for (int i = 0; i < n_curves; ++i) {
        PK_CURVE_t curve = curves[i];
        double t_min = bounds[i].value[0];
        double t_max = bounds[i].value[1];

        std::vector<double> tmp_curve;
        tmp_curve.reserve(samples_per_curve * 3);

        for (int s = 0; s < samples_per_curve; ++s) {
            double t = t_min + (t_max - t_min) * s / (samples_per_curve - 1);
            PK_VECTOR_t pt;
            PK_CURVE_eval(curve, t, 0, &pt);  // 只取点，不求导数 [citation:3]
            tmp_curve.push_back(pt.coord[0]);
            tmp_curve.push_back(pt.coord[1]);
            tmp_curve.push_back(pt.coord[2]);
        }
        intersection_points.push_back(std::move(tmp_curve));
    }

    // 4. 释放 Parasolid 内部分配的内存 [citation:10]
    if (vectors) PK_MEMORY_free(vectors);
    if (curves)  PK_MEMORY_free(curves);
    if (bounds)  PK_MEMORY_free(bounds);
    if (types)   PK_MEMORY_free(types);

    // 5. 删除临时曲面
    PK_ENTITY_delete(1, &bsurf1);
    PK_ENTITY_delete(1, &bsurf2);
}
vector<std::vector<Point>> NurbsPlaneOpenCurveInt(double planA, double planB, double planC, float plane_normal[3], ON_NurbsSurface nurbs_surf)
{


    std::vector<std::vector<Point>> viewoutput;
  
    int n, m, p, q;

    // 获取阶数
    int degree_u = nurbs_surf.Degree(0);
    int degree_v = nurbs_surf.Degree(1);
    // 获取控制点数
    int cv_count_u = nurbs_surf.CVCount(0);
    int cv_count_v = nurbs_surf.CVCount(1);

    // 计算 n, m (根据NURBS标准定义)
    n = cv_count_u - 1;  // 最后一个控制点的U索引
    m = cv_count_v - 1;  // 最后一个控制点的V索引
    p = degree_u;
    q = degree_v;
   
    // 获取U方向节点
    int u_count = nurbs_surf.KnotCount(0) + 2;
    std::vector<double> KnotU(u_count);
    KnotU[0] = 0;
    KnotU[u_count - 1] = 1;
    for (int i = 0; i < (u_count - 2); i++) {
        KnotU[i + 1] = nurbs_surf.Knot(0, i);
    }

    // 获取V方向节点
    int v_count = nurbs_surf.KnotCount(1) + 2;
    std::vector<double> KnotV(v_count);
    KnotV[0] = 0;
    KnotV[v_count - 1] = 1;
    for (int i = 0; i < (v_count - 2); i++) {
        KnotV[i + 1] = nurbs_surf.Knot(1, i);
    }
   
    std::vector<double> mul_u;
    std::vector<double> mul_v;
    /////////////////////////////////////////////////////////////////////////提取节点向量重数


    std::vector<double> distinct_knots_u;
    std::vector<double> distinct_knots_v;
    // 获取第一个节点
    double current_knot = nurbs_surf.Knot(0, 0);
    distinct_knots_u.push_back(current_knot);

    int count = 1;

    // 遍历所有节点
    for (int i = 1; i < u_count; i++) {
        double knot = nurbs_surf.Knot(0, i);

        if (fabs(knot - current_knot) < ON_ZERO_TOLERANCE) {
            count++;
        }
        else {
            // 保存前一个节点的重数
            mul_u.push_back(static_cast<double>(count));

            // 开始新的节点
            current_knot = knot;
            distinct_knots_u.push_back(current_knot);
            count = 1;
        }
    }

    // 保存最后一个节点的重数
    mul_u.push_back(static_cast<double>(count));


    // 提取V方向节点向量和重数




    current_knot = nurbs_surf.Knot(1, 0);
    distinct_knots_v.push_back(current_knot);

    count = 1;

    for (int i = 1; i < v_count; i++) {
        double knot = nurbs_surf.Knot(1, i);

        if (fabs(knot - current_knot) < ON_ZERO_TOLERANCE) {
            count++;
        }
        else {
            mul_v.push_back(static_cast<double>(count));
            current_knot = knot;
            distinct_knots_v.push_back(current_knot);
            count = 1;
        }
    }

    mul_v.push_back(static_cast<double>(count));
 

    std::vector<double> U;
    std::vector<double> V;
    double minUknot = KnotU[0];
    double maxUknot = KnotU[KnotU.size() - 1];
    double minVknot = KnotV[0];
    double maxVknot = KnotV[KnotV.size() - 1];

    for (int i = 0; i < KnotU.size(); i++)
    {

        U.push_back(KnotU[i]);

    }
    for (int i = 0; i < KnotV.size(); i++)
    {

        V.push_back(KnotV[i]);

    }



    /*使用Opennurbs计算NURBS曲面nurbs_surf上的点及k阶偏导
    double val[18];
    nurbs_surf.Evaluate(para_u, para_v, k, 3, val);*/

    //####################################################
    //######################################################假设平面的法向量和一个平面上的点
    //***************************************************
    //Vector3d normal(A, B, C);
    //Vector3d point(a, b, c);
    double A = plane_normal[0];
    double B = plane_normal[1];
    double C = plane_normal[2];
    Vector3d normal(A, B, C);
    Vector3d point(planA, planB, planC);
    //double firstZ = point[2];

    // 计算仿射变换矩阵
    Matrix4d affineMatrix = computeAffineMatrix(normal, point);

    // 定义原始点
    Point p0 = { 1.0, 0.0, 0.0 };
    Matrix4x4 transform = {
       {affineMatrix
       }
    };


    //// 进行变换
    //Point transformedPoint = transform.multiply(p0);

    // 输出结果
   /* cout << "Original Point: (" << p0.x << ", " << p0.y << ", " << p0.z << ")" << endl;
    cout << "Transformed Point: (" << transformedPoint.x << ", " << transformedPoint.y << ", " << transformedPoint.z << ")" << endl;*/





    ON::Begin(); // 初始化OpenNURBS库
   // ON_NurbsSurface nurbs_surf;
    ON_NurbsSurface nurbs_surf1;
    /*  CreatNURBS(p, q, n, m, Pw, U, V, nurbs_surf);*/

      /*Normalize control point*/
    ON_3dPointArray control_points;
    for (int i = 0; i < nurbs_surf.CVCount(0); i++)
    {
        for (int j = 0; j < nurbs_surf.CVCount(1); j++)
        {
            ON_3dPoint control_point;
            nurbs_surf.GetCV(i, j, control_point);
            control_points.Append(control_point);
        }
    }


    // 构造一个包围盒
    ON_Point min_pt(0.0, 0.0, 0.0);
    ON_Point max_pt(1.0, 1.0, 1.0);
    ON_BoundingBox bbox(min_pt, max_pt);
    ON_BoundingBox bboxt(min_pt, max_pt);

    // 获取控制点数组的范围
    bboxt.Set(control_points);

    Eigen::MatrixXd ControlPnt_PX(n + 1, m + 1);
    Eigen::MatrixXd ControlPnt_PY(n + 1, m + 1);
    Eigen::MatrixXd ControlPnt_PZ(n + 1, m + 1);
    std::vector<Eigen::MatrixXd> Pw;

    ON_3dPoint cv1;
    for (int i = 0; i < nurbs_surf.CVCount(0); i++)
    {
        for (int j = 0; j < nurbs_surf.CVCount(1); j++)
        {

            nurbs_surf.GetCV(i, j, cv1);

            ControlPnt_PX(i, j) = cv1.x;
            ControlPnt_PY(i, j) = cv1.y;
            ControlPnt_PZ(i, j) = cv1.z;
        }
    }


    Pw.push_back(ControlPnt_PX);
    Pw.push_back(ControlPnt_PY);
    Pw.push_back(ControlPnt_PZ);

    vector<Eigen::MatrixXd> Pwt = TransformControlPoints(n, m, Pw, transform);
    /*  std::cout << "Pwt: " << "\n";
      for (int i = 0; i <= n; i++)
      {
          for (int j = 0; j <= m; j++)
          {
              std::cout << Pwt[0](i, j) << " " << Pwt[1](i, j) << " " << Pwt[2](i, j) << "   ";
          }
          std::cout << " " << "\n";
      }
      std::cout << "\n";*/
    CreatNURBS(p, q, n, m, Pwt, U, V, nurbs_surf1);
    //归一化曲面
    Vector3d transPoint = SurfaceNormalization(nurbs_surf, bbox, point);

    //归一化曲面
    Vector3d transPoint1 = SurfaceNormalizationt(nurbs_surf1, bbox, point);

    // 设置你想计算的u和v参数（比如曲面中点，u=0.5，v=0.5）
    double u = 0.5;
    double v = 0.5;

    // 确保曲面已成功创建
    if (nurbs_surf.IsValid()) {
        //std::cerr << "NURBS surface is valid!" << std::endl;
    }
    int dir = 1;
    // std::cout << "The Boundary1" << "\n";
    double point234[6];
   // double point432[6];
    //// 记录开始时间点
    //auto start = std::chrono::high_resolution_clock::now();


    ///*******************************************************
    // 求取曲面上u, v参数对应的点
   // ON_3dPoint point1 = GetPointOnNURBSSurface(nurbs_surf, u, v);

    // 输出计算出的点的坐标
    /*std::cout << "Point at (u=" << u << ", v=" << v << "): "
        << point1.x << ", " << point1.y << ", " << point1.z << std::endl;*/
        /* ***********************************************************
          查找并输出 z = 0 的点*/
          //FindPointsWithZZero(nurbs_surf,transform);



    //// 记录开始时间点
    auto start = std::chrono::high_resolution_clock::now();

    //求曲面内的点 
    std::vector<std::vector<double>> para_uv;
    //int pqtest = ConvexHullOnSplinedSurface(nurbs_surf1);
    //int pqtest = NormalCone(nurbs_surf1);


    int pqtest = ConvexHullOnSurface(nurbs_surf1);
    /* std::cout << "曲面是否含有极点" << pqtest << "\n";
     std::cout << "The Surface:" << "\n";*/

     /*std::cout << "p = SplitTheSurface:" << p << "\n";*/

    vector<double> para_point;
    //小环存储向量
   // std::vector<Point> loop;
    /*double u0 = 0.25;
    double v0 = 0.25;
    std::vector<double> tIntersect;
    bool testw = gradient_descent(u0, v0, 0.0005, 1000000, nurbs_surf1, tIntersect);
    if (testw) {
        for (int i = 0; i < tIntersect.size(); i++) {
            ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, tIntersect[0], tIntersect[1]);
            Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
            loop.push_back(point001);
        }
    }*/

    //test area#########################################################################################
    // #################################################################################################
      // 提取一条等参线 (u 方向)
    std::vector<double> para_1, para_2, para_3, para_4;
    //double v_position = 0.0 * (nurbs_surf1.Domain(1).Min() + nurbs_surf1.Domain(1).Max());  // 
    ON_Curve* iso_curve1 = nurbs_surf1.IsoCurve(0, minUknot);  // 0 表示 u 方向等参线
    ON_Curve* iso_curve2 = nurbs_surf1.IsoCurve(0, maxUknot);
    ON_Curve* iso_curve3 = nurbs_surf1.IsoCurve(1, minVknot);
    ON_Curve* iso_curve4 = nurbs_surf1.IsoCurve(1, maxVknot);

    //输出的点列
    std::vector<Point> Boundary1;
    std::vector<Point> Boundary2;
    std::vector<Point> Boundary3;
    std::vector<Point> Boundary4;


   // int values[] = { 1 ,-1 };
    vector<int>values= { 1 ,-1 };



    int ptest = ConvexHull(*iso_curve1);
   if (ptest == 1) { Boundaryintersection(1,iso_curve1, para_1, point234, viewoutput, nurbs_surf, values, nurbs_surf1, Boundary1, minUknot, minVknot, maxUknot, maxVknot); }
    ptest = ConvexHull(*iso_curve2);
    if (ptest == 1) { Boundaryintersection(2,iso_curve2, para_2, point234, viewoutput, nurbs_surf, values, nurbs_surf1, Boundary2, minUknot, minVknot, maxUknot, maxVknot); }
    ptest = ConvexHull(*iso_curve3);
    if (ptest == 1) { Boundaryintersection(3,iso_curve3, para_3, point234, viewoutput, nurbs_surf, values, nurbs_surf1, Boundary3, minUknot, minVknot, maxUknot, maxVknot); }
    ptest = ConvexHull(*iso_curve4);
    if (ptest == 1) { Boundaryintersection(4,iso_curve4, para_4, point234, viewoutput, nurbs_surf, values, nurbs_surf1, Boundary4, minUknot, minVknot, maxUknot, maxVknot); }
 

 

    ////边界求交记录结束时间点
     //auto end = std::chrono::high_resolution_clock::now();

     //// 计算时间差并转换为毫秒
     //std::chrono::duration<double, std::milli> elapsed = end - start;

     //std::cout << "边界求交检测时间: " << elapsed.count() << " 毫秒" << std::endl;




    int i;
    int j;
    Point pointOut;



    //还原
    double xmin = bboxt.m_min.x;
    double ymin = bboxt.m_min.y;
    double zmin = bboxt.m_min.z;
    double xmax = bboxt.m_max.x;
    double ymax = bboxt.m_max.y;
    double zmax = bboxt.m_max.z;


    double cube_size = 1.0;
    double max_size = (std::max)(xmax - xmin, ymax - ymin);
    max_size = (std::max)(max_size, zmax - zmin);

    double a = transPoint[0];
    double b = transPoint[1];
    double c = transPoint[2];
    for (i = 0; i < viewoutput.size(); i++) {
        for (j = 0; j < viewoutput[i].size(); j++) {
            pointOut = viewoutput[i][j];
            pointOut.x = (pointOut.x + transPoint[0]) * max_size / cube_size + xmin;
            pointOut.y = (pointOut.y + transPoint[1]) * max_size / cube_size + ymin;
            pointOut.z = (pointOut.z + transPoint[2]) * max_size / cube_size + zmin;
            viewoutput[i][j] = pointOut;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差并转换为毫秒
    std::chrono::duration<double, std::milli> elapsed = end - start;

    std::cout << "求交总时间: " << elapsed.count() << " 毫秒" << std::endl;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////


    std::cout << "para_size: " << para_1.size() << ", " << para_2.size() << ", " << para_3.size() << ", " << para_4.size() << "\n";
    std::cout << "para_point: " << para_point.size() << "\n" << "para_uv(0,0,1)" << para_uv.size();


    //     // 遍历每一层，查看该层的交点个数（分支数）
    //for (size_t i = 0; i < viewoutput.size(); i++) {
    //    size_t num_branches = viewoutput[i].size();
    //    std::cout << "第 " << i << " 层的分支个数: " << num_branches << std::endl;

    //    // 如果需要，还可以查看每个分支的具体坐标
    //    for (size_t j = 0; j < viewoutput[i].size(); j++) {
    //        std::cout << "  分支 " << j << ": " << viewoutput[i][j].x << viewoutput[i][j].y << viewoutput[i][j].z << std::endl;
    //    }
    //}
    return viewoutput;
}

vector<std::vector<Point>> NurbsPlaneLoopInt(double planA, double planB, double planC, float plane_normal[3], ON_NurbsSurface nurbs_surf)
{


    std::vector<std::vector<Point>> viewoutput;
    ///////////////////////////////////////////////
    // ////////////////////////////////////////////
    //std::ifstream SURF;
    //SURF.open("D:\\安装文件\\project\\example\\B-spline_SelfIntFree_1.txt");//输入文件
    ////SURF.open("D:\\安装文件\\project\\example\\曲面数据\\B-spline_Boat.txt");//输入文件
    // //SURF.open("D:\\安装文件\\project\\example\\spline.txt");


    // //degree:(p,q)
    int n, m, p, q;

    // 获取阶数
    int degree_u = nurbs_surf.Degree(0);
    int degree_v = nurbs_surf.Degree(1);
    // 获取控制点数
    int cv_count_u = nurbs_surf.CVCount(0);
    int cv_count_v = nurbs_surf.CVCount(1);

    // 计算 n, m (根据NURBS标准定义)
    n = cv_count_u - 1;  // 最后一个控制点的U索引
    m = cv_count_v - 1;  // 最后一个控制点的V索引
    p = degree_u;
    q = degree_v;
    //SURF >> n;
    //SURF >> m;
    //SURF >> p;
    //SURF >> q;
    ///*  std::cout << "p: " << p << "\n";
    //  std::cout << "q: " << q << "\n";*/



    //double con = 1;
    //int rr1 = 1, rr2 = 1;
    ////int rr1 = 3, rr2 = 5;

    ////Knot :KnotU,KnotV
  /*  std::vector<double> KnotU;
    std::vector<double> KnotV;*/
    // 获取U方向节点
    int u_count = nurbs_surf.KnotCount(0) + 2;
    std::vector<double> KnotU(u_count);
    KnotU[0] = 0;
    KnotU[u_count - 1] = 1;
    for (int i = 0; i < (u_count - 2); i++) {
        KnotU[i + 1] = nurbs_surf.Knot(0, i);
    }

    // 获取V方向节点
    int v_count = nurbs_surf.KnotCount(1) + 2;
    std::vector<double> KnotV(v_count);
    KnotV[0] = 0;
    KnotV[v_count - 1] = 1;
    for (int i = 0; i < (v_count - 2); i++) {
        KnotV[i + 1] = nurbs_surf.Knot(1, i);
    }
    //for (int i = 0; i <= n - p - rr1 + 2; i++)
    //{
    //    double val;
    //    SURF >> val;
    //    KnotU.push_back(val);
    //}
    //for (int i = 0; i <= m - q - rr2 + 2; i++)
    //{
    //    double val;
    //    SURF >> val;
    //    KnotV.push_back(val);
    //}
    ///*for (int i = 0; i <= 114; i++)
    //{
    //    double val;
    //    SURF >> val;
    //    KnotV.push_back(val);
    //}*/
    ////multiplicity :mul_u,mul_v
    std::vector<double> mul_u;
    std::vector<double> mul_v;
    /////////////////////////////////////////////////////////////////////////提取节点向量重数


    std::vector<double> distinct_knots_u;
    std::vector<double> distinct_knots_v;
    // 获取第一个节点
    double current_knot = nurbs_surf.Knot(0, 0);
    distinct_knots_u.push_back(current_knot);

    int count = 1;

    // 遍历所有节点
    for (int i = 1; i < u_count; i++) {
        double knot = nurbs_surf.Knot(0, i);

        if (fabs(knot - current_knot) < ON_ZERO_TOLERANCE) {
            count++;
        }
        else {
            // 保存前一个节点的重数
            mul_u.push_back(static_cast<double>(count));

            // 开始新的节点
            current_knot = knot;
            distinct_knots_u.push_back(current_knot);
            count = 1;
        }
    }

    // 保存最后一个节点的重数
    mul_u.push_back(static_cast<double>(count));


    // 提取V方向节点向量和重数




    current_knot = nurbs_surf.Knot(1, 0);
    distinct_knots_v.push_back(current_knot);

    count = 1;

    for (int i = 1; i < v_count; i++) {
        double knot = nurbs_surf.Knot(1, i);

        if (fabs(knot - current_knot) < ON_ZERO_TOLERANCE) {
            count++;
        }
        else {
            mul_v.push_back(static_cast<double>(count));
            current_knot = knot;
            distinct_knots_v.push_back(current_knot);
            count = 1;
        }
    }

    mul_v.push_back(static_cast<double>(count));
    //////////////////////////////////////////////////////////////////////////////////////////////////

        //for (int i = 0; i < KnotU.size(); i++)
        //{
        //    double val;
        //    SURF >> val;
        //    mul_u.push_back(val);
        //}
        //for (int i = 0; i < KnotV.size(); i++)
        //{
        //    double val;
        //    SURF >> val;
        //    mul_v.push_back(val);
        //}

        //Eigen::MatrixXd ControlPnt_PX(n + 1, m + 1);
        //Eigen::MatrixXd ControlPnt_PY(n + 1, m + 1);
        //Eigen::MatrixXd ControlPnt_PZ(n + 1, m + 1);
        ////Control points: Pw
        //std::vector<Eigen::MatrixXd> Pw;
        //for (int i = 0; i <= n; i++)
        //{
        //    for (int j = 0; j <= m; j++)
        //    {
        //        double pnt0, pnt1, pnt2;
        //        SURF >> pnt0;
        //        SURF >> pnt1;
        //        SURF >> pnt2;
        //        ControlPnt_PX(i, j) = con * pnt0;
        //        ControlPnt_PY(i, j) = con * pnt1;
        //        ControlPnt_PZ(i, j) = con * pnt2;
        //    }
        //}


        //Pw.push_back(ControlPnt_PX);
        //Pw.push_back(ControlPnt_PY);
        //Pw.push_back(ControlPnt_PZ);


        ///* std::cout << "KnotU: " << "\n";
        // for (int i = 0; i < KnotU.size(); i++)
        // {

        //     std::cout << KnotU[i] << " ";

        // }
        // std::cout << "\n";
        // std::cout << "KnotV: " << "\n";
        // for (int i = 0; i < KnotV.size(); i++)
        // {

        //     std::cout << KnotV[i] << " ";

        // }
        // std::cout << "\n";
        // std::cout << "mul_u: " << "\n";
        // for (int i = 0; i < KnotU.size(); i++)
        // {

        //     std::cout << mul_u[i] << " ";

        // }
        // std::cout << "\n";
        // std::cout << "mul_v: " << "\n";
        // for (int i = 0; i < KnotV.size(); i++)
        // {

        //     std::cout << mul_v[i] << " ";

        // }
        // std::cout << "\n";*/
        // /* std::cout << "Pw: " << "\n";
        //  for (int i = 0; i <= n; i++)
        //  {
        //      for (int j = 0; j <= m; j++)
        //      {
        //          std::cout << Pw[0](i, j) << " " << Pw[1](i, j) << " " << Pw[2](i, j) << "   ";
        //      }
        //      std::cout << " " << "\n";
        //  }
        //  std::cout << "\n";*/

        //  //KnotVector :U,V
    std::vector<double> U;
    std::vector<double> V;
    double minUknot = KnotU[0];
    double maxUknot = KnotU[KnotU.size() - 1];
    double minVknot = KnotV[0];
    double maxVknot = KnotV[KnotV.size() - 1];

    for (int i = 0; i < KnotU.size(); i++)
    {

        U.push_back(KnotU[i]);

    }
    for (int i = 0; i < KnotV.size(); i++)
    {

        V.push_back(KnotV[i]);

    }
    ///* std::cout << "U: " << "\n";
    // for (int i = 0; i < U.size(); i++)
    // {

    //     std::cout << U[i] << " ";

    // }
    // std::cout << "\n";
    // std::cout << "V: " << "\n";
    // for (int i = 0; i < V.size(); i++)
    // {

    //     std::cout << V[i] << " ";

    // }
    // std::cout << "\n";*/


    //time_t TotalTime_begin, TotalTime_end, Determine_1_begin, Determine_1_end, B_spline_begin, B_spline_end, Bezier_begin, Bezier_end;
    //TotalTime_begin = clock();
    //std::vector<double> Int_Pnts;
    //std::vector<double> Start_Pnts;


//////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////读取曲面部分


   /* double planA = 0;
    double planB = 0;
    double planC = -2;*/



    /*使用Opennurbs计算NURBS曲面nurbs_surf上的点及k阶偏导
    double val[18];
    nurbs_surf.Evaluate(para_u, para_v, k, 3, val);*/

    //####################################################
    //######################################################假设平面的法向量和一个平面上的点
    //***************************************************
    //Vector3d normal(A, B, C);
    //Vector3d point(a, b, c);
    double A = plane_normal[0];
    double B = plane_normal[1];
    double C = plane_normal[2];
    Vector3d normal(A, B, C);
    Vector3d point(planA, planB, planC);
    //double firstZ = point[2];

    // 计算仿射变换矩阵
    Matrix4d affineMatrix = computeAffineMatrix(normal, point);




    //std::cout << "Affine Transformation Matrix:\n" << affineMatrix << std::endl;

    //// 原始平面上的一个点
    //Point3D p0 = { 1.0, 1.0, 3.0};
    ////检测点是否在平面上
    //if ((a * p0.x + b * p0.y + c * p0.z+d) !=0) {
    //    std::cout << "Point is not on the plant " << std::endl;
    //}

    //// 生成旋转矩阵
    //vector<vector<double>> rotationMatrix = generateRotationMatrix(a, b, c);

    //// 计算平移量，使平面移动到 z=0 位置
    //double translationZ = d / c;
    //vector<double> translation = { 0, 0, -translationZ };

    //// 对点进行仿射变换
    //Point3D transformedPoint = affineTransform(p0, rotationMatrix, translation);

    // 输出结果
  /*  std::cout << "原始点: (" << p0.x << ", " << p0.y << ", " << p0.z << ")" << endl;
    std::cout << "变换后的点: (" << transformedPoint.x << ", " << transformedPoint.y << ", " << transformedPoint.z << ")" << endl;*/

    // 定义原始点
    Point p0 = { 1.0, 0.0, 0.0 };
    Matrix4x4 transform = {
       {affineMatrix
       }
    };


    //// 进行变换
    //Point transformedPoint = transform.multiply(p0);

    // 输出结果
   /* cout << "Original Point: (" << p0.x << ", " << p0.y << ", " << p0.z << ")" << endl;
    cout << "Transformed Point: (" << transformedPoint.x << ", " << transformedPoint.y << ", " << transformedPoint.z << ")" << endl;*/





    ON::Begin(); // 初始化OpenNURBS库
   // ON_NurbsSurface nurbs_surf;
    ON_NurbsSurface nurbs_surf1;
    /*  CreatNURBS(p, q, n, m, Pw, U, V, nurbs_surf);*/

      /*Normalize control point*/
    ON_3dPointArray control_points;
    for (int i = 0; i < nurbs_surf.CVCount(0); i++)
    {
        for (int j = 0; j < nurbs_surf.CVCount(1); j++)
        {
            ON_3dPoint control_point;
            nurbs_surf.GetCV(i, j, control_point);
            control_points.Append(control_point);
        }
    }



    //int cv_count_u = nurbs_surf.CVCount(0);  // U 方向的控制点数量
    //int cv_count_v = nurbs_surf.CVCount(1);  // V 方向的控制点数量
    //for (int i = 0; i < cv_count_u; ++i) {
    //    for (int j = 0; j < cv_count_v; ++j) {
        //  ON_3dPoint cv_point;
    //        nurbs_surf.GetCV(i, j, cv_point);
    //        std::cout << "CV[" << i << "][" << j << "] = (" << cv_point.x << ", " << cv_point.y << ", " << cv_point.z << ")" << std::endl;
    //    }
    //}

    // 构造一个包围盒
    ON_Point min_pt(0.0, 0.0, 0.0);
    ON_Point max_pt(1.0, 1.0, 1.0);
    ON_BoundingBox bbox(min_pt, max_pt);
    ON_BoundingBox bboxt(min_pt, max_pt);

    // 获取控制点数组的范围
    bboxt.Set(control_points);

    Eigen::MatrixXd ControlPnt_PX(n + 1, m + 1);
    Eigen::MatrixXd ControlPnt_PY(n + 1, m + 1);
    Eigen::MatrixXd ControlPnt_PZ(n + 1, m + 1);
    std::vector<Eigen::MatrixXd> Pw;

    ON_3dPoint cv1;
    for (int i = 0; i < nurbs_surf.CVCount(0); i++)
    {
        for (int j = 0; j < nurbs_surf.CVCount(1); j++)
        {

            nurbs_surf.GetCV(i, j, cv1);

            ControlPnt_PX(i, j) = cv1.x;
            ControlPnt_PY(i, j) = cv1.y;
            ControlPnt_PZ(i, j) = cv1.z;
        }
    }


    Pw.push_back(ControlPnt_PX);
    Pw.push_back(ControlPnt_PY);
    Pw.push_back(ControlPnt_PZ);

    vector<Eigen::MatrixXd> Pwt = TransformControlPoints(n, m, Pw, transform);
    /*  std::cout << "Pwt: " << "\n";
      for (int i = 0; i <= n; i++)
      {
          for (int j = 0; j <= m; j++)
          {
              std::cout << Pwt[0](i, j) << " " << Pwt[1](i, j) << " " << Pwt[2](i, j) << "   ";
          }
          std::cout << " " << "\n";
      }
      std::cout << "\n";*/
    CreatNURBS(p, q, n, m, Pwt, U, V, nurbs_surf1);
    //归一化曲面
    Vector3d transPoint = SurfaceNormalization(nurbs_surf, bbox, point);

    //归一化曲面
    Vector3d transPoint1 = SurfaceNormalizationt(nurbs_surf1, bbox, point);

    // 设置你想计算的u和v参数（比如曲面中点，u=0.5，v=0.5）
    double u = 0.5;
    double v = 0.5;

    // 确保曲面已成功创建
    if (nurbs_surf.IsValid()) {
        //std::cerr << "NURBS surface is valid!" << std::endl;
    }
    int dir = 1;
    // std::cout << "The Boundary1" << "\n";
    double point234[6];
    double point432[6];
    //// 记录开始时间点
    //auto start = std::chrono::high_resolution_clock::now();


    ///*******************************************************
    // 求取曲面上u, v参数对应的点
   // ON_3dPoint point1 = GetPointOnNURBSSurface(nurbs_surf, u, v);

    // 输出计算出的点的坐标
    /*std::cout << "Point at (u=" << u << ", v=" << v << "): "
        << point1.x << ", " << point1.y << ", " << point1.z << std::endl;*/
        /* ***********************************************************
          查找并输出 z = 0 的点*/
          //FindPointsWithZZero(nurbs_surf,transform);



    //// 记录开始时间点
    auto start = std::chrono::high_resolution_clock::now();

    //求曲面内的点 
    std::vector<std::vector<double>> para_uv;
    //int pqtest = ConvexHullOnSplinedSurface(nurbs_surf1);
    //int pqtest = NormalCone(nurbs_surf1);


    int pqtest = ConvexHullOnSurface(nurbs_surf1);
    /* std::cout << "曲面是否含有极点" << pqtest << "\n";
     std::cout << "The Surface:" << "\n";*/

     /*std::cout << "p = SplitTheSurface:" << p << "\n";*/

    vector<double> para_point;
    //小环存储向量
    std::vector<Point> loop;
    /*double u0 = 0.25;
    double v0 = 0.25;
    std::vector<double> tIntersect;
    bool testw = gradient_descent(u0, v0, 0.0005, 1000000, nurbs_surf1, tIntersect);
    if (testw) {
        for (int i = 0; i < tIntersect.size(); i++) {
            ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, tIntersect[0], tIntersect[1]);
            Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
            loop.push_back(point001);
        }
    }*/

    if (pqtest == 1) {


        int p = SplitTheSurface(nurbs_surf1, para_uv);
        //int delnumber = delthepoint(para_uv);
        int tengent = 0;
        for (int i = 0; i < para_uv.size(); i++) {
            ON_3dPoint original_point002 = GetPointOnNURBSSurface(nurbs_surf1, para_uv[i][0], para_uv[i][1]);

            if (abs(original_point002[2]) < 1e-4) {
                ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, para_uv[i][0], para_uv[i][1]);
                Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
                loop.push_back(point001);
                para_uv.erase(para_uv.begin() + i);
                tengent = tengent + 1;
            }
        }

        //// 边界求交记录结束时间点
        //auto end = std::chrono::high_resolution_clock::now();

        //// 计算时间差并转换为毫秒
        //std::chrono::duration<double, std::milli> elapsed = end - start;

        //std::cout << "闭环检测时间: " << elapsed.count() << " 毫秒" << std::endl;


        // std::cout << "para_uv: " << "\n";

        for (int i = 0; i < para_uv.size(); i++) {
            /*for (int j = i + 1; j < para_uv.size(); j++) {
                double dis = abs(para_uv[i][0] - para_uv[j][0]) * abs(para_uv[i][0] - para_uv[j][0]) + abs(para_uv[i][1] - para_uv[j][1]) * abs(para_uv[i][1] - para_uv[j][1]);
                if (dis > 1e-3) {*/

                //std::cout << para_uv[i][0] << " ,  " << para_uv[i][1];
            ON_Curve* iso_curvepoint = nurbs_surf1.IsoCurve(0, para_uv[i][1]);  // 表示顶点处 u 方向等参线


            ON_3dPoint point2 = GetPointOnNURBSSurface(nurbs_surf, para_uv[i][0], para_uv[i][1]);
            // std::cout << "法向为（0,0,1）的点：" << point2[0] << " " << point2[1] << " " << point2[2] << "\n";
            int k = para_uv.size();
            Point point_ansuv = { point2[0] ,point2[1] ,point2[2] };
            //  loop.push_back(point_ansuv);


              /* std::cout << "法向为（0,0,1）的点对应等参线：" << "\n";*/


            SplitTheinnerBoundary(iso_curvepoint, para_point);
            /* for (int k = 0; k < para_point.size(); k++) {
                 ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, para_point[k], para_uv[k][1]);
                 Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
                 loop.push_back(point001);
             }*/
             //删除重复的parade_point
             ////////////////////////////////////////////////
             ///还没更改！！！！
            /* for (int i = 0; i < para_point.size(); i++) {
                 ON_3dPoint original_point002 = GetPointOnNURBSSurface(nurbs_surf1, para_uv[i][0], para_uv[i][1]);

                 if (abs(original_point002[2]) < 1e-4) {
                     ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, para_uv[i][0], para_uv[i][1]);
                     Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
                     loop.push_back(point001);
                     para_uv.erase(para_uv.begin() + i);

                 }
             }*/
             ///////////////////////////////
             // ////////////////////////////
               ////输出求得小环上的起始点
               //std::cout << "para_point: " << "\n";
               //for (int d = 0; d < para_point.size(); d++) {
               //  /*  std::cout << para_point[d] << " ,  ";*/

               //    ON_3dPoint point001 = GetPointOnNURBSSurface(nurbs_surf, para_point[d], para_uv[i][1]);
               //   // Point point_ans001 = { point001[0] ,point001[1] ,point001[2] };
               //    //loop.push_back(point_ans001);
               //    std::cout <<"("<< point001[0] << " ,  " << point001[1] << " ,  " << point001[2]<<")";
               //    std::cout << "\n";
               //    //std::cout << "points on 法向为（0,0,1）的点对应等参线: " << "\n";
               //}





            for (int k = 0; k < para_point.size(); k++) {
                iso_curvepoint->Evaluate(para_point[k], 1, 3, point432);
                if (abs(point432[2]) < 1e-4) {
                    /*std::cout << "(" << point432[0] << "," << point432[1] << "," << point432[2] << ")";
                    std::cout << "\n";*/
                    Point point_ans = { point432[0] ,point432[1] ,point432[2] };
                    if (whethertraced(viewoutput, point_ans) == 0) {
                        //Point original_point = inverse_affine_transform(point_ans, affineMatrix);
                        ON_3dPoint original_point432 = GetPointOnNURBSSurface(nurbs_surf, para_point[k], para_uv[i][1]);
                        Point original_point = { original_point432[0] ,original_point432[1] ,original_point432[2] };
                        /* cout << "逆变换后的点: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                         std::cout << "trace出的点： " << "\n";*/
                        loop.push_back(original_point);
                        dir = 1;

                        vector<double> p1 = tracelineloop(nurbs_surf1, para_point[k], para_uv[i][1], dir);
                        double p11 = para_uv[i][1];
                        double p10 = para_point[k];
                        for (int j = 0; j < 1000 && p1[0] <= maxUknot && p1[1] <= maxVknot && p1[0] >= minUknot && p1[1] >= minVknot && whetherlooped(p1, p10, p11) == 0; j++) {//每一个点trace出来的点数

                            ON_3dPoint point234 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
                            // if (abs(point234[2]) < 1e-4) {
                                // std::cout << "(" << point234[0] << "," << point234[1] << "," << point234[2] << ")";
                           /* std::cout << "\n";*/
                            Point point_ans = { point234[0] ,point234[1] ,point234[2] };

                            /*  Point original_point = inverse_affine_transform(point_ans, affineMatrix);*/
                              // cout << "trace的点逆变换后为: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                             /* cout << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;*/
                              //}
                              //else {
                                //  cout << "所求点不满足要求。" << endl;
                              //}
                            loop.push_back(point_ans);
                            p1 = tracelineloop(nurbs_surf1, p1[0], p1[1], dir);
                        }
                        /* double dis = abs(p1[0] - original_point.x) + abs(p1[1] - original_point.y) + abs(p1[2] - original_point.z);
                             if (dis < 1e-3) {
                                 loop.push_back(original_point);
                             }*/
                             // int Loopsize = loop.size();
                        viewoutput.push_back(loop);
                        loop.clear();

                        //dir = -1;
                        //loop.push_back(original_point);
                        //p1 = tracelineloop(nurbs_surf1, para_point[k], para_uv[i][0], dir);
                        //for (int j = 0; j < 1000 && p1[0] <= maxUknot && p1[1] <= maxVknot && p1[0] >= minUknot && p1[1] >= minVknot && whetherlooped(p1, p10, p11) == 0; j++) {//每一个点trace出来的点数
                        //    ON_3dPoint point234 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
                        //    // if (abs(point234[2]) < 1e-4) {
                        //        // std::cout << "(" << point234[0] << "," << point234[1] << "," << point234[2] << ")";
                        //   /* std::cout << "\n";*/
                        //    Point point_ans = { point234[0] ,point234[1] ,point234[2] };
                        //    if (whethertraced(viewoutput, point_ans) == 0) {

                        //        /*  Point original_point = inverse_affine_transform(point_ans, affineMatrix);*/
                        //          // cout << "trace的点逆变换后为: " << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;
                        //         /* cout << original_point.x << " " << original_point.y << " " << original_point.z << " " << endl;*/
                        //          //}
                        //          //else {
                        //            //  cout << "所求点不满足要求。" << endl;
                        //          //}
                        //        loop.push_back(point_ans);
                        //        p1 = tracelineloop(nurbs_surf1, p1[0], p1[1], dir);
                        //    }
                        //} 
                        /* dis = abs(p1[0] - original_point.x) + abs(p1[1] - original_point.y) + abs(p1[2] - original_point.z);
                        if (dis < 1e-3) {
                            loop.push_back(original_point);
                        }*/
                        // int Loopsize = loop.size();
                        //viewoutput.push_back(loop);
                      //  loop.clear();

                    }
                }
            }

        }
    }





    
    int i;
    int j;
    Point pointOut;



    //还原
    double xmin = bboxt.m_min.x;
    double ymin = bboxt.m_min.y;
    double zmin = bboxt.m_min.z;
    double xmax = bboxt.m_max.x;
    double ymax = bboxt.m_max.y;
    double zmax = bboxt.m_max.z;


    double cube_size = 1.0;
    double max_size = (std::max)(xmax - xmin, ymax - ymin);
    max_size = (std::max)(max_size, zmax - zmin);

    double a = transPoint[0];
    double b = transPoint[1];
    double c = transPoint[2];
    for (i = 0; i < viewoutput.size(); i++) {
        for (j = 0; j < viewoutput[i].size(); j++) {
            pointOut = viewoutput[i][j];
            pointOut.x = (pointOut.x + transPoint[0]) * max_size / cube_size + xmin;
            pointOut.y = (pointOut.y + transPoint[1]) * max_size / cube_size + ymin;
            pointOut.z = (pointOut.z + transPoint[2]) * max_size / cube_size + zmin;
            viewoutput[i][j] = pointOut;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差并转换为毫秒
    std::chrono::duration<double, std::milli> elapsed = end - start;

    std::cout << "求交总时间: " << elapsed.count() << " 毫秒" << std::endl;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////




    //     // 遍历每一层，查看该层的交点个数（分支数）
    //for (size_t i = 0; i < viewoutput.size(); i++) {
    //    size_t num_branches = viewoutput[i].size();
    //    std::cout << "第 " << i << " 层的分支个数: " << num_branches << std::endl;

    //    // 如果需要，还可以查看每个分支的具体坐标
    //    for (size_t j = 0; j < viewoutput[i].size(); j++) {
    //        std::cout << "  分支 " << j << ": " << viewoutput[i][j].x << viewoutput[i][j].y << viewoutput[i][j].z << std::endl;
    //    }
    //}
    return viewoutput;

  


}

vector<std::vector<Point>> NurbsPlaneInt2(double planA, double planB, double planC, float plane_normal[3], ON_NurbsSurface nurbs_surf,int ModelNumber)
{


    std::vector<std::vector<Point>> viewoutput;
    std::vector<std::vector<Point>> viewtracepoint;
    std::vector<std::vector<Point>> viewopencurve;
    std::vector<std::vector<Point>> viewloop;
   

    // //degree:(p,q)
    int n, m, p, q;

    // 获取阶数
    int degree_u = nurbs_surf.Degree(0);
    int degree_v = nurbs_surf.Degree(1);
    // 获取控制点数
    int cv_count_u = nurbs_surf.CVCount(0);
    int cv_count_v = nurbs_surf.CVCount(1);

    // 计算 n, m (根据NURBS标准定义)
    n = cv_count_u - 1;  // 最后一个控制点的U索引
    m = cv_count_v - 1;  // 最后一个控制点的V索引
    p = degree_u;
    q = degree_v;
    
    // 获取U方向节点
    int u_count = nurbs_surf.KnotCount(0) + 2;
    std::vector<double> KnotU(u_count);
    KnotU[0] = 0;
    KnotU[u_count - 1] = 1;
    for (int i = 0; i < (u_count - 2); i++) {
        KnotU[i + 1] = nurbs_surf.Knot(0, i);
    }

    // 获取V方向节点
    int v_count = nurbs_surf.KnotCount(1) + 2;
    std::vector<double> KnotV(v_count);
    KnotV[0] = 0;
    KnotV[v_count - 1] = 1;
    for (int i = 0; i < (v_count - 2); i++) {
        KnotV[i + 1] = nurbs_surf.Knot(1, i);
    }
   
    std::vector<double> mul_u;
    std::vector<double> mul_v;
    /////////////////////////////////////////////////////////////////////////提取节点向量重数


    std::vector<double> distinct_knots_u;
    std::vector<double> distinct_knots_v;
    // 获取第一个节点
    double current_knot = nurbs_surf.Knot(0, 0);
    distinct_knots_u.push_back(current_knot);

    int count = 1;

    // 遍历所有节点
    for (int i = 1; i < u_count; i++) {
        double knot = nurbs_surf.Knot(0, i);

        if (fabs(knot - current_knot) < ON_ZERO_TOLERANCE) {
            count++;
        }
        else {
            // 保存前一个节点的重数
            mul_u.push_back(static_cast<double>(count));

            // 开始新的节点
            current_knot = knot;
            distinct_knots_u.push_back(current_knot);
            count = 1;
        }
    }

    // 保存最后一个节点的重数
    mul_u.push_back(static_cast<double>(count));


    // 提取V方向节点向量和重数




    current_knot = nurbs_surf.Knot(1, 0);
    distinct_knots_v.push_back(current_knot);

    count = 1;

    for (int i = 1; i < v_count; i++) {
        double knot = nurbs_surf.Knot(1, i);

        if (fabs(knot - current_knot) < ON_ZERO_TOLERANCE) {
            count++;
        }
        else {
            mul_v.push_back(static_cast<double>(count));
            current_knot = knot;
            distinct_knots_v.push_back(current_knot);
            count = 1;
        }
    }

    mul_v.push_back(static_cast<double>(count));

    std::vector<double> U;
    std::vector<double> V;
    double minUknot = KnotU[0];
    double maxUknot = KnotU[KnotU.size() - 1];
    double minVknot = KnotV[0];
    double maxVknot = KnotV[KnotV.size() - 1];

    for (int i = 0; i < KnotU.size(); i++)
    {

        U.push_back(KnotU[i]);

    }
    for (int i = 0; i < KnotV.size(); i++)
    {

        V.push_back(KnotV[i]);

    }
   


//////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////读取曲面部分


   /* double planA = 0;
    double planB = 0;
    double planC = -2;*/



    /*使用Opennurbs计算NURBS曲面nurbs_surf上的点及k阶偏导
    double val[18];
    nurbs_surf.Evaluate(para_u, para_v, k, 3, val);*/

    //####################################################
    //######################################################假设平面的法向量和一个平面上的点
    //***************************************************
    //Vector3d normal(A, B, C);
    //Vector3d point(a, b, c);
    double A = plane_normal[0];
    double B = plane_normal[1];
    double C = plane_normal[2];
    Vector3d normal(A, B, C);
    Vector3d point(planA, planB, planC);
    //double firstZ = point[2];

    // 计算仿射变换矩阵
    Matrix4d affineMatrix = computeAffineMatrix(normal, point);


    // 定义原始点
    Point p0 = { 1.0, 0.0, 0.0 };
    Matrix4x4 transform = {
       {affineMatrix
       }
    };



    ON::Begin(); // 初始化OpenNURBS库
   // ON_NurbsSurface nurbs_surf;
    ON_NurbsSurface nurbs_surf1;
    /*  CreatNURBS(p, q, n, m, Pw, U, V, nurbs_surf);*/

      /*Normalize control point*/
    ON_3dPointArray control_points;
    for (int i = 0; i < nurbs_surf.CVCount(0); i++)
    {
        for (int j = 0; j < nurbs_surf.CVCount(1); j++)
        {
            ON_3dPoint control_point;
            nurbs_surf.GetCV(i, j, control_point);
            control_points.Append(control_point);
        }
    }



    //int cv_count_u = nurbs_surf.CVCount(0);  // U 方向的控制点数量
    //int cv_count_v = nurbs_surf.CVCount(1);  // V 方向的控制点数量
    //for (int i = 0; i < cv_count_u; ++i) {
    //    for (int j = 0; j < cv_count_v; ++j) {
        //  ON_3dPoint cv_point;
    //        nurbs_surf.GetCV(i, j, cv_point);
    //        std::cout << "CV[" << i << "][" << j << "] = (" << cv_point.x << ", " << cv_point.y << ", " << cv_point.z << ")" << std::endl;
    //    }
    //}

    // 构造一个包围盒
    ON_Point min_pt(0.0, 0.0, 0.0);
    ON_Point max_pt(1.0, 1.0, 1.0);
    ON_BoundingBox bbox(min_pt, max_pt);
    ON_BoundingBox bboxt(min_pt, max_pt);

    // 获取控制点数组的范围
    bboxt.Set(control_points);

    Eigen::MatrixXd ControlPnt_PX(n + 1, m + 1);
    Eigen::MatrixXd ControlPnt_PY(n + 1, m + 1);
    Eigen::MatrixXd ControlPnt_PZ(n + 1, m + 1);
    std::vector<Eigen::MatrixXd> Pw;

    ON_3dPoint cv1;
    for (int i = 0; i < nurbs_surf.CVCount(0); i++)
    {
        for (int j = 0; j < nurbs_surf.CVCount(1); j++)
        {

            nurbs_surf.GetCV(i, j, cv1);

            ControlPnt_PX(i, j) = cv1.x;
            ControlPnt_PY(i, j) = cv1.y;
            ControlPnt_PZ(i, j) = cv1.z;
        }
    }


    Pw.push_back(ControlPnt_PX);
    Pw.push_back(ControlPnt_PY);
    Pw.push_back(ControlPnt_PZ);

    vector<Eigen::MatrixXd> Pwt = TransformControlPoints(n, m, Pw, transform);
    /*  std::cout << "Pwt: " << "\n";
      for (int i = 0; i <= n; i++)
      {
          for (int j = 0; j <= m; j++)
          {
              std::cout << Pwt[0](i, j) << " " << Pwt[1](i, j) << " " << Pwt[2](i, j) << "   ";
          }
          std::cout << " " << "\n";
      }
      std::cout << "\n";*/
    CreatNURBS(p, q, n, m, Pwt, U, V, nurbs_surf1);
    //归一化曲面
    Vector3d transPoint = SurfaceNormalization(nurbs_surf, bbox, point);

    //归一化曲面
    Vector3d transPoint1 = SurfaceNormalizationt(nurbs_surf1, bbox, point);

    // 设置你想计算的u和v参数（比如曲面中点，u=0.5，v=0.5）
    double u = 0.5;
    double v = 0.5;

    // 确保曲面已成功创建
    if (nurbs_surf.IsValid()) {
        //std::cerr << "NURBS surface is valid!" << std::endl;
    }
    int dir = 1;
    // std::cout << "The Boundary1" << "\n";
    double point234[6];
    double point432[6];
    //// 记录开始时间点
    //auto start = std::chrono::high_resolution_clock::now();


    ///*******************************************************
    // 求取曲面上u, v参数对应的点
   // ON_3dPoint point1 = GetPointOnNURBSSurface(nurbs_surf, u, v);

    // 输出计算出的点的坐标
    /*std::cout << "Point at (u=" << u << ", v=" << v << "): "
        << point1.x << ", " << point1.y << ", " << point1.z << std::endl;*/
        /* ***********************************************************
          查找并输出 z = 0 的点*/
          //FindPointsWithZZero(nurbs_surf,transform);



    //// 记录开始时间点
    auto start = std::chrono::high_resolution_clock::now();

    //求曲面内的点 
    std::vector<std::vector<double>> para_uv;
    //int pqtest = ConvexHullOnSplinedSurface(nurbs_surf1);
    //int pqtest = NormalCone(nurbs_surf1);


    int pqtest = ConvexHullOnSurface(nurbs_surf1);
    /* std::cout << "曲面是否含有极点" << pqtest << "\n";
     std::cout << "The Surface:" << "\n";*/
     /*std::cout << "p = SplitTheSurface:" << p << "\n";*/

    vector<double> para_point;
    //小环存储向量
    std::vector<Point> loop;
    std::vector<Point> looppoint;
    if (pqtest == 1) {

        int p = SplitTheSurface(nurbs_surf1, para_uv);
        //int delnumber = delthepoint(para_uv);
        int tengent = 0;
        for (int i = 0; i < para_uv.size(); i++) {
            
            ON_3dPoint original_point002 = GetPointOnNURBSSurface(nurbs_surf1, para_uv[i][0], para_uv[i][1]);
            

           if (abs(original_point002[2]) < 1e-4) {
                ON_3dPoint original_point001 = GetPointOnNURBSSurface(nurbs_surf, para_uv[i][0], para_uv[i][1]);
                Point point001 = { original_point001[0] ,original_point001[1] ,original_point001[2] };
                looppoint.push_back(point001);
                para_uv.erase(para_uv.begin() + i);
                tengent = tengent + 1;
            }
        }

      

        for (int i = 0; i < para_uv.size(); i++) {
            /*for (int j = i + 1; j < para_uv.size(); j++) {
                double dis = abs(para_uv[i][0] - para_uv[j][0]) * abs(para_uv[i][0] - para_uv[j][0]) + abs(para_uv[i][1] - para_uv[j][1]) * abs(para_uv[i][1] - para_uv[j][1]);
                if (dis > 1e-3) {*/

                //std::cout << para_uv[i][0] << " ,  " << para_uv[i][1];
            ON_Curve* iso_curvepoint = nurbs_surf1.IsoCurve(0, para_uv[i][1]);  // 表示顶点处 u 方向等参线


            ON_3dPoint point2 = GetPointOnNURBSSurface(nurbs_surf, para_uv[i][0], para_uv[i][1]);
            // std::cout << "法向为（0,0,1）的点：" << point2[0] << " " << point2[1] << " " << point2[2] << "\n";
            int k = para_uv.size();
            Point point_ansuv = { point2[0] ,point2[1] ,point2[2] };
            //  loop.push_back(point_ansuv);


              /* std::cout << "法向为（0,0,1）的点对应等参线：" << "\n";*/

            int innerlinetest = whetherlineontheplane(iso_curvepoint, 1e-4);
            if (innerlinetest == 1) {
                //边界在平面上，把边界输出
                outputlineontheplane(iso_curvepoint, viewopencurve);
            }
            else {
                SplitTheinnerBoundary(iso_curvepoint, para_point);
            }

           
            if (ModelNumber == 1) {
                for (int k = 0; k < para_point.size(); k++) {
               
                    iso_curvepoint->Evaluate(para_point[k], 1, 3, point432);
                    if (abs(point432[2]) < 1e-4) {
                        Point point_ans = { point432[0] ,point432[1] ,point432[2] };

                        ON_3dPoint original_point432 = GetPointOnNURBSSurface(nurbs_surf, para_point[k], para_uv[i][1]);
                        Point original_point = { original_point432[0] ,original_point432[1] ,original_point432[2] };
                        loop.push_back(original_point);
                     // viewtracepoint.push_back(looppoint);
                      viewtracepoint.push_back(loop);
                        loop.clear();

                    }
                }
            }else{
                for (int k = 0; k < para_point.size(); k++) {
                    iso_curvepoint->Evaluate(para_point[k], 1, 3, point432);
                    if (abs(point432[2]) < 1e-4) {
                        Point point_ans = { point432[0] ,point432[1] ,point432[2] };
                        if (whethertraced(viewloop, point_ans) == 0) {
                           // if (whethertraced(viewoutput, point_ans) == 0) {
                            ON_3dPoint original_point432 = GetPointOnNURBSSurface(nurbs_surf, para_point[k], para_uv[i][1]);
                            Point original_point = { original_point432[0] ,original_point432[1] ,original_point432[2] };
                            loop.push_back(original_point);
                            dir = 1;
                            vector<double> p1 = tracelineloop(nurbs_surf1, para_point[k], para_uv[i][1], dir);
                            double p11 = para_uv[i][1];
                            double p10 = para_point[k];
                            for (int j = 0; j < 1000 && p1[0] <= maxUknot && p1[1] <= maxVknot && p1[0] >= minUknot && p1[1] >= minVknot && whetherlooped(p1, p10, p11) == 0; j++) {//每一个点trace出来的点数
                                ON_3dPoint point234 = GetPointOnNURBSSurface(nurbs_surf, p1[0], p1[1]);
                                Point point_ans = { point234[0] ,point234[1] ,point234[2] };
                                loop.push_back(point_ans);
                                p1 = tracelineloop(nurbs_surf1, p1[0], p1[1], dir);
                            }
                           // viewoutput.push_back(loop);
                            viewloop.push_back(loop);
                            loop.clear();
                        }
                    }
                }
            }

  

        }
    }





    //test area#########################################################################################
    // #################################################################################################
      // 提取一条等参线 (u 方向)
    std::vector<double> para_1, para_2, para_3, para_4;
    //double v_position = 0.0 * (nurbs_surf1.Domain(1).Min() + nurbs_surf1.Domain(1).Max());  // 
    ON_Curve* iso_curve1 = nurbs_surf1.IsoCurve(0, minUknot);  // 0 表示 u 方向等参线
    ON_Curve* iso_curve2 = nurbs_surf1.IsoCurve(0, maxUknot);
    ON_Curve* iso_curve3 = nurbs_surf1.IsoCurve(1, minVknot);
    ON_Curve* iso_curve4 = nurbs_surf1.IsoCurve(1, maxVknot);

    //输出的点列
    std::vector<Point> Boundary1;
    std::vector<Point> Boundary2;
    std::vector<Point> Boundary3;
    std::vector<Point> Boundary4;

    // int values[] = { 1 ,-1 };
    vector<int>values = { 1 ,-1 };



    int ptest = ConvexHull(*iso_curve1);
    int linetest = whetherlineontheplane(iso_curve1, 1e-4);
    if (linetest == 1) {
   //边界在平面上，把边界输出
         outputlineontheplane(iso_curve1, viewopencurve);
    }
    else {
        if (ptest == 1) { Boundaryintersection(1, iso_curve1, para_1, point234, viewopencurve, nurbs_surf, values, nurbs_surf1, Boundary1, minUknot, minVknot, maxUknot, maxVknot); }
    }
    


    ptest = ConvexHull(*iso_curve2);
    linetest = whetherlineontheplane(iso_curve2, 1e-4);
    if (linetest == 1) {
        //边界在平面上，把边界输出
        outputlineontheplane(iso_curve2, viewopencurve);
    }
    else {
        if (ptest == 1) { Boundaryintersection(2, iso_curve2, para_2, point234, viewopencurve, nurbs_surf, values, nurbs_surf1, Boundary2, minUknot, minVknot, maxUknot, maxVknot); }
    }



    ptest = ConvexHull(*iso_curve3);
    linetest = whetherlineontheplane(iso_curve3, 1e-4);
    if (linetest == 1) {
        //边界在平面上，把边界输出
        outputlineontheplane(iso_curve3, viewopencurve);
    }
    else {
        if (ptest == 1) { Boundaryintersection(3, iso_curve3, para_3, point234, viewopencurve, nurbs_surf, values, nurbs_surf1, Boundary3, minUknot, minVknot, maxUknot, maxVknot); }
    }



    ptest = ConvexHull(*iso_curve4);
    linetest = whetherlineontheplane(iso_curve4, 1e-4);
    if (linetest == 1) {
        //边界在平面上，把边界输出
        outputlineontheplane(iso_curve4, viewopencurve);
    }
    else {
        if (ptest == 1) { Boundaryintersection(4, iso_curve4, para_4, point234, viewopencurve, nurbs_surf, values, nurbs_surf1, Boundary4, minUknot, minVknot, maxUknot, maxVknot); }
    }

    int i;
    int j;
    Point pointOut;



    //还原
    double xmin = bboxt.m_min.x;
    double ymin = bboxt.m_min.y;
    double zmin = bboxt.m_min.z;
    double xmax = bboxt.m_max.x;
    double ymax = bboxt.m_max.y;
    double zmax = bboxt.m_max.z;


    double cube_size = 1.0;
    double max_size = (std::max)(xmax - xmin, ymax - ymin);
    max_size = (std::max)(max_size, zmax - zmin);

    double a = transPoint[0];
    double b = transPoint[1];
    double c = transPoint[2];
    if (ModelNumber == 1) { viewoutput = viewtracepoint; }
    if (ModelNumber == 2) { viewoutput = viewopencurve;}
    if (ModelNumber ==3 ) { viewoutput = viewloop; }
    if (ModelNumber == 0) {
        viewoutput.insert(viewoutput.end(), viewloop.begin(), viewloop.end()); viewoutput.insert(viewoutput.end(), viewopencurve.begin(), viewopencurve.end());
    }
    for (i = 0; i < viewoutput.size(); i++) {
        for (j = 0; j < viewoutput[i].size(); j++) {
            pointOut = viewoutput[i][j];
            pointOut.x = (pointOut.x + transPoint[0]) * max_size / cube_size + xmin;
            pointOut.y = (pointOut.y + transPoint[1]) * max_size / cube_size + ymin;
            pointOut.z = (pointOut.z + transPoint[2]) * max_size / cube_size + zmin;
            viewoutput[i][j] = pointOut;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差并转换为毫秒
    std::chrono::duration<double, std::milli> elapsed = end - start;

    std::cout << "求交总时间: " << elapsed.count() << " 毫秒" << std::endl;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////


    //     // 遍历每一层，查看该层的交点个数（分支数）
    //for (size_t i = 0; i < viewoutput.size(); i++) {
    //    size_t num_branches = viewoutput[i].size();
    //    std::cout << "第 " << i << " 层的分支个数: " << num_branches << std::endl;

    //    // 如果需要，还可以查看每个分支的具体坐标
    //    for (size_t j = 0; j < viewoutput[i].size(); j++) {
    //        std::cout << "  分支 " << j << ": " << viewoutput[i][j].x << viewoutput[i][j].y << viewoutput[i][j].z << std::endl;
    //    }
    //}
    return viewoutput;

  


}

int Maptraceline(vector<std::vector<Point>> groups) {
    // ================= 内部参数 =================
  
        constexpr int   W = 1024;
        constexpr int   H = 1024;
        constexpr int   POINT_RADIUS = 1;
 
   

        std::string path = savePointsImage(groups, POINT_RADIUS,  W, H);
        // path = "./output/points_20250101_120000.png"
        return 0;
   
}