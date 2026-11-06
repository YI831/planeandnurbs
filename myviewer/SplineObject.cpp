#include "SplineObject.h"

// NURBS
void mtriangulation(const ON_NurbsSurface& surf, easy3d::SurfaceMesh& mesh, int ResolutionU, int ResolutionV)
{
	std::vector<easy3d::SurfaceMesh::Vertex> vertices((ResolutionU + 1) * (ResolutionV + 1));
	double umin = surf.Domain(0).Min();
	double umax = surf.Domain(0).Max();
	double vmin = surf.Domain(1).Min();
	double vmax = surf.Domain(1).Max();
	double ustep = (umax - umin) / double(ResolutionU);
	double vstep = (vmax - vmin) / double(ResolutionV);

	double uparam = umin;
	double vparam = vmin;

	for (int i = 0; i <= ResolutionU; i++)
	{
		vparam = vmin;
		for (int j = 0; j <= ResolutionV; j++)
		{
			ON_3dPoint p = surf.PointAt(uparam, vparam);
			auto v = mesh.add_vertex(easy3d::vec3(p.x, p.y, p.z));
			vertices[i * (ResolutionV + 1) + j] = v;

			vparam += vstep;
		}
		uparam += ustep;
	}
	for (int i = 0; i < ResolutionU; i++)
		for (int j = 0; j < ResolutionV; j++)
		{
			mesh.add_triangle(vertices[i * (ResolutionV + 1) + j], vertices[(i + 1) * (ResolutionV + 1) + j], vertices[i * (ResolutionV + 1) + j + 1]);
			mesh.add_triangle(vertices[i * (ResolutionV + 1) + j + 1], vertices[(i + 1) * (ResolutionV + 1) + j], vertices[(i + 1) * (ResolutionV + 1) + j + 1]);
		}
}

NurbsObject::NurbsObject(const ON_NurbsSurface& surface) 
{
	controlNet = ControlNetObject();
	vList.clear();

    // control points
	double* cv = surface.m_cv;
	int idx = 0;
    int uctrl = surface.m_cv_count[0];
    int vctrl = surface.m_cv_count[1];
	for (int ui = 0; ui < uctrl; ++ui) 
	{
		vList.push_back(std::vector<easy3d::Graph::Vertex>());
		for (int vi = 0; vi < vctrl; ++vi) 
		{
			easy3d::Graph::Vertex vp = controlNet.add_vertex(easy3d::vec3(cv[idx],cv[idx + 1],cv[idx + 2]));
			vList[ui].push_back(vp);
			idx += 3;
		}
	}
	for (int ui = 0; ui < uctrl; ++ui)
	{
		for (int vi = 0; vi < vctrl - 1; ++vi)
		{
			controlNet.add_edge(vList[ui][vi], vList[ui][vi + 1]);
		}
	}
	for (int vi = 0; vi < vctrl; ++vi)
	{
		for (int ui = 0; ui < uctrl - 1; ++ui)
		{
			controlNet.add_edge(vList[ui][vi], vList[ui + 1][vi]);
		}
	}

	splineSurf = surface;
	mtriangulation(splineSurf, *this, meshResolutionU, meshResolutionV);
}

void NurbsObject::updateTriangulation() 
{
	clear();
	mtriangulation(splineSurf, *this, meshResolutionU, meshResolutionV);
}

NurbsObject* ReadNurbs(std::string file) 
{
    ON_NurbsSurface* surf = nullptr;
    bool success = false;
    std::ifstream fin;
    fin.open(file);
    std::string line;

    while (std::getline(fin, line)) 
    {
        if (line.substr(0, 12) == "Bspline Surf") 
        {
            int u_degree, v_degree, u_ctrl, v_ctrl;
            std::istringstream lineIn(line);
            std::string word;
            lineIn >> word;
            lineIn >> word;
            lineIn >> word;
            lineIn >> word;
            lineIn >> u_degree;
            lineIn >> word;
            lineIn >> word;
            lineIn >> v_degree;
            lineIn >> word;
            lineIn >> word;
            lineIn >> u_ctrl;
            lineIn >> word;
            lineIn >> word;
            lineIn >> v_ctrl;
            surf = ON_NurbsSurface::New(3, false, u_degree + 1, v_degree + 1, u_ctrl, v_ctrl);
            for (int i = 0; i < u_ctrl; ++i) 
            {
                std::getline(fin, line);
                lineIn.clear();
                lineIn.str(line);
                for (int j = 0; j < v_ctrl; ++j) 
                {
                    ON_3dPoint cp;
                    lineIn >> cp[0];
                    lineIn >> cp[1];
                    lineIn >> cp[2];
                    surf->SetCV(i, j, cp);
                    if (lineIn.eof() && j != v_ctrl - 1)
                    {
                        std::getline(fin, line);
                        lineIn.clear();
                        lineIn.str(line);
                    }
                }
            }
            std::getline(fin, line);
            while (line.substr(0, 13) != "Knot Sequence") 
            {
                std::getline(fin, line);
            }
            int u_knots = u_ctrl + u_degree - 1;
            int v_knots = v_ctrl + v_degree - 1;
            double*& knotsU = surf->m_knot[0];
            double*& knotsV = surf->m_knot[1];
            fin >> word;
            for (int i = 0; i < u_knots; ++i) 
            {
                fin >> knotsU[i];
            }
            fin >> word;
            fin >> word;
            for (int i = 0; i < v_knots; ++i) 
            {
                fin >> knotsV[i];
            }
            fin >> word;
        }
        else if (line.substr(0, 10) == "NURBS Surf")
        {
            int u_degree, v_degree, u_ctrl, v_ctrl;
            std::istringstream lineIn(line);
            std::string word;
            lineIn >> word;
            lineIn >> word;
            lineIn >> word;
            lineIn >> word;
            lineIn >> u_degree;
            lineIn >> word;
            lineIn >> word;
            lineIn >> v_degree;
            lineIn >> word;
            lineIn >> word;
            lineIn >> u_ctrl;
            lineIn >> word;
            lineIn >> word;
            lineIn >> v_ctrl;
            surf = ON_NurbsSurface::New(3, true, u_degree + 1, v_degree + 1, u_ctrl, v_ctrl);
            for (int i = 0; i < u_ctrl; ++i) 
            {
                std::getline(fin, line);
                lineIn.clear();
                lineIn.str(line);
                for (int j = 0; j < v_ctrl; ++j) 
                {
                    ON_4dPoint cp;
                    lineIn >> cp[0];
                    lineIn >> cp[1];
                    lineIn >> cp[2];
                    lineIn >> cp[3];
                    surf->SetCV(i, j, cp);
                    if (lineIn.eof() && j != v_ctrl - 1)
                    {
                        std::getline(fin, line);
                        lineIn.clear();
                        lineIn.str(line);
                    }
                }
            }
            std::getline(fin, line);
            while (line.substr(0, 13) != "Knot Sequence") 
            {
                std::getline(fin, line);
            }
            int u_knots = u_ctrl + u_degree - 1;
            int v_knots = v_ctrl + v_degree - 1;
            double*& knotsU = surf->m_knot[0];
            double*& knotsV = surf->m_knot[1];
            fin >> word;
            for (int i = 0; i < u_knots; ++i)
            {
                fin >> knotsU[i];
            }
            fin >> word;
            fin >> word;
            for (int i = 0; i < v_knots; ++i)
            {
                fin >> knotsV[i];
            }
            fin >> word;
        }
    }
    fin.close();
    NurbsObject* nurbsObj = new NurbsObject(*surf);
    surf->Destroy();
    surf = nullptr;

    return nurbsObj;
}

///////////////////////////////////////////////////////////用与构造occ的平面
ON_NurbsSurface CreatePlane(float plane_point[3], float plane_normal[3], double size) {
    ON_3dPoint P;
    P.x = plane_point[0];
    P.y = plane_point[1];
    P.z = plane_point[2];
    ON_3dVector N(plane_normal[0], plane_normal[1], plane_normal[2]);
    // 1. 法向量归一化
    ON_3dVector normal = N;
    normal.Unitize();

    // 2. 计算局部坐标系
    ON_3dVector U, V;
    if (fabs(normal.x) > fabs(normal.y)) {
        U = ON_CrossProduct(normal, ON_3dVector(normal.z, 0, -normal.x));
    }
    else {
        U = ON_CrossProduct(normal, ON_3dVector(0, normal.z, -normal.y));
    }
    U.Unitize();
    V = ON_CrossProduct(normal, U);
    V.Unitize();

    // 3. 四个角点
    double s = size * 0.5;
    ON_3dPoint P00 = P - s * U - s * V;
    ON_3dPoint P10 = P + s * U - s * V;
    ON_3dPoint P01 = P - s * U + s * V;
    ON_3dPoint P11 = P + s * U + s * V;

    // 4. 【关键修正】先创建曲面，再设置属性
    ON_NurbsSurface surface;

    // 创建双线性NURBS曲面（次数1,1，2x2控制点）
    bool success = surface.Create(3,     // 3维点
        false, // 非有理曲面（无权值）
        2,     // u方向次数
        2,     // v方向次数
        2,     // u方向控制点数
        2);    // v方向控制点数

    if (!success) {
        // 创建失败处理
        std::cout << "平面创建失败！ " << "\n"; // 或抛出异常
    }

    // 5. 设置节点向量
    // u方向节点
    surface.SetKnot(0, 0, 0.0);
    surface.SetKnot(0, 1, 1.0);
    surface.SetKnot(0, 2, 1.0);
    surface.SetKnot(0, 3, 1.0);
    surface.SetKnot(0, 4, 1.0);
    surface.SetKnot(0, 5, 1.0);

    // v方向节点
    surface.SetKnot(1, 0, 0.0);
    surface.SetKnot(1, 1, 1.0);
    surface.SetKnot(1, 2, 1.0);
    surface.SetKnot(1, 3, 1.0);
    surface.SetKnot(1, 4, 1.0);
    surface.SetKnot(1, 5, 1.0);

    // 6. 设置控制点
    surface.SetCV(0, 0, P00);  // 索引从0开始
    surface.SetCV(1, 0, P10);
    surface.SetCV(0, 1, P01);
    surface.SetCV(1, 1, P11);

    // 验证
    // ASSERT(surface.CVCount(0) == 2);
    // ASSERT(surface.CVCount(1) == 2);

    return surface;
}