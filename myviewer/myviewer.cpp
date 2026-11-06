#include "myviewer.h"
#include <easy3d/util/dialogs.h>
#include <easy3d/util/file_system.h>

#include "3rd_party/imgui/imgui.h"
#include "3rd_party/imgui/misc/fonts/imgui_fonts_droid_sans.h"
#include "3rd_party/imgui/backends/imgui_impl_glfw.h"
#include "3rd_party/imgui/backends/imgui_impl_opengl3.h"

#include "SplineObject.h"
#include "parasolid_kernel.h"

#include "NurbsPlaneInt/test.h"
using namespace easy3d;

struct MyModel
{
    MyModel() :surf(nullptr), ctrlnet(nullptr), plane(nullptr), insec(nullptr), singlepoint(nullptr), insec_occ(nullptr), singlepoint_occ(nullptr) {}

    void clear()
    {
        surf = nullptr;
        ctrlnet = nullptr;
        plane = nullptr;
        insec = nullptr;
        singlepoint = nullptr;
        insec_occ = nullptr;
        singlepoint_occ = nullptr;
    }

    easy3d::SurfaceMesh* surf;
    easy3d::Graph* ctrlnet; // control net of surface
    easy3d::SurfaceMesh* plane;
    easy3d::Graph* insec;
    easy3d::Graph* singlepoint;

    //occ
    easy3d::Graph* insec_occ;
    easy3d::Graph* singlepoint_occ;
};



MyModel mymod;

// parameters for viewer
bool drawsurf = true;
bool drawplane = true;
bool drawctrl = false;
bool drawctrlpts = false;
bool drawinsec = true;
bool drawinsec_occ = true;

// parameter for plane
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
float plane_point[3] = { 0,0,0 };
float plane_normal[3] = { 0,0,1 };

float half_w = 5;
float half_h = 5;



MyViewer::MyViewer(
    const std::string& title /* = "Easy3D ImGui Viewer" */,
    int samples /* = 4 */,
    int gl_major /* = 3 */,
    int gl_minor /* = 2 */,
    bool full_screen /* = false */,
    bool resizable /* = true */,
    int depth_bits /* = 24 */,
    int stencil_bits /* = 8 */
)
    : ViewerImGui(title, samples, gl_major, gl_minor, full_screen, resizable, depth_bits, stencil_bits) {}

// mouse and key events

bool MyViewer::key_press_event(int key, int modifiers)
{
    return ViewerImGui::key_press_event(key, modifiers);
}

inline bool MyViewer::mouse_press_event(int x, int y, int button, int modifiers)
{
    if (modifiers == EASY3D_MOD_CONTROL)
    {
        return true;
    }
    else
    {
        return Viewer::mouse_press_event(x, y, button, modifiers);
    }
}

inline bool MyViewer::mouse_release_event(int x, int y, int button, int modifiers)
{
    if (modifiers == EASY3D_MOD_CONTROL)
    {
        return true;
    }
    else
        return Viewer::mouse_release_event(x, y, button, modifiers);
}

inline bool MyViewer::mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers)
{
    if (modifiers == EASY3D_MOD_CONTROL)
    {
        return true;
    }
    else
    {
        return Viewer::mouse_drag_event(x, y, dx, dy, button, modifiers);
    }
}

void MyViewer::initrenderer()
{
    
    if (mymod.surf)
    {
        fit_screen(mymod.surf);
        mymod.surf->renderer()->set_visible(drawsurf);
    }
    // 修改曲面颜色
    auto surf_faces = mymod.surf->renderer()->get_triangles_drawable("faces");
    if (surf_faces) {
        surf_faces->set_uniform_coloring(easy3d::vec4(100 / 255.0f, 180 / 255.0f, 255 / 255.0f, 1.0f));
        surf_faces->state().set_back_color(easy3d::vec4(100 / 255.0f, 180 / 255.0f, 255 / 255.0f, 1.0f)); // #64B4FF 晴空蓝


    }
    if (mymod.ctrlnet)
    {
        // ctrl grid
        mymod.ctrlnet->renderer()->set_visible(drawctrl);

        // ctrl pts
        mymod.ctrlnet->renderer()->get_points_drawable("vertices")->set_visible(drawctrlpts);
        mymod.ctrlnet->renderer()->get_points_drawable("vertices")->set_impostor_type(easy3d::PointsDrawable::PLAIN);
        fit_screen(mymod.surf);
        mymod.surf->renderer()->set_visible(drawsurf);
      
    
    }
}
void MyViewer::initmodel(const std::vector<std::string>& files)
{
    // delete model
    
    for (int i = models_.size() - 1; i >= 0; i--)
    {
        auto p = models_[0];
        delete_model(p);
      
    }
    mymod.clear();//原来这里只写了mymod.plane=nullptr和mymod.insec=nullptr,改成mymod.clear就好了
    // load models
    for (std::string pth : files)
    {
        const std::string& ext = easy3d::file_system::extension(pth, true);
        if (ext == "nurbs") // bspline patch
        {
            NurbsObject* nurbsobj = ReadNurbs(pth);
            if (nurbsobj)
            {
                std::string name = easy3d::file_system::base_name(pth);
                nurbsobj->set_name(name);
                add_model(nurbsobj);

                ControlNetObject* ctrlobj = new ControlNetObject(nurbsobj->controlNet);
                ctrlobj->set_name(name + "_ControlNet");
                add_model(ctrlobj);

                mymod.surf = nurbsobj;
                mymod.ctrlnet = ctrlobj;
              
            }
        }
        else // other type
        {
            mymod.surf = dynamic_cast<decltype(mymod.surf)>(add_model(pth));
        }
        if (mymod.surf == nullptr)
        {
            std::cout << "Cannot open: " << pth << std::endl;
            ::exit(1);
        }
    }

    
    // set renderer
    initrenderer();
}


void MyViewer::intersection()
{
    
    // surf
    NurbsObject* nurbsobj = dynamic_cast<NurbsObject*>(mymod.surf);

    if (!nurbsobj)
        return;
    //nurbsobj -> splineSurf;
    int i;
    int j;
  
   // ON_NurbsSurface nurbs_surf = *(casgeo::casBspline2opNurbs(nurbsobj->splineSurf));

    //int loopsize;
   // vector<std::vector<Point>>  pointout = NurbsPlaneInt(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf);
    vector<std::vector<Point>>  pointout = NurbsPlaneInt2(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf,0);
  
    // 遍历每一层，查看该层的交点个数（分支数）
    //for (size_t i = 3; i < 4; i++) {
    //    size_t num_branches = pointout[i].size();
    //    std::cout << "/n第 " << i+1 << " 层的分支个数: " << num_branches << std::endl;

    //    // 如果需要，还可以查看每个分支的具体坐标
    //    for (size_t j = 0; j < pointout[i].size(); j++) {
    //        if (abs(pointout[i][j].z - plane_point[2]) > 0.0001) {
    //            std::cout << pointout[i][j].x << " " << pointout[i][j].y << " " << pointout[i][j].z << " ,";
    //            std::cout <<"i:" << i << " j:" << j;
    //        }
    //        std::cout << pointout[i][j].x <<" " << pointout[i][j].y << " " << pointout[i][j].z << " ,";
    //    }
    //}
    /*for (i = 0; i < pointout.size(); i++) {
        loopsize = pointout[i].size();
        if (loopsize == 1) {
            Point p_0 = pointout[i][0];
            pointout[i].resize(2);
            pointout[i][0] = p_0;
            Point p_1 = { p_0.x + 1e-6,p_0.y + 1e-6,p_0.z + 1e-6 };
            pointout[i][1] = p_1;
        }
    }*/
    //////////////occ测试
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //   vector<std::vector<Point>>  pointout;
 //   Point pointpush;
 //   std::vector<std::vector<double>> intersection_points ;

 //   ON_NurbsSurface occPlane = CreatePlane(plane_point, plane_normal, 40);
 //   nurbsintersection_OCC( nurbsobj->splineSurf, occPlane, intersection_points);
 ///*   for (size_t i = 0; i < intersection_points.size(); ++i) {
 //       std::cout << "i:" << i<< std::endl;
 //       int intsize = intersection_points[i].size();
 //       for (j = 0; j < intersection_points[i].size(); j++) {
 //           std::cout  << intersection_points[i][j]<<"，" ;
 //       }
 //  
 //   }*/
 //   std::vector<Point> line;


 //   for (i = 0; i < intersection_points.size(); i++) {
 //       for (j = 0; j < intersection_points[i].size()/3; j++) {
 //           pointpush.x = intersection_points[i][3*j+0];
 //           pointpush.y = intersection_points[i][3*j+1];
 //           pointpush.z = intersection_points[i][3*j+2];
 //           line.push_back(pointpush);
 //       }
 //       pointout.push_back(line);
 //       line.clear();
 //   }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //vector<std::vector<Point>>  singlepointout;//输出单个点用于检测
    //Point Ptest = {0,0,0};
    //vector<Point> Pt = {Ptest};
    //singlepointout.push_back(Pt);
    easy3d::Graph* intersection_result = new easy3d::Graph();
    easy3d::Graph* singleintersection_result = new easy3d::Graph();
    for (i = 0; i < pointout.size(); i++)
    {
        std::vector<easy3d::vec3> output; //假设这是你的输出
        for (const auto& p : pointout[i]) {
            output.emplace_back(p.x, p.y, p.z);
        }
        if (output.size() > 0) 
        {
            if (output.size() == 1) //single point
            {

                singleintersection_result->add_vertex(easy3d::vec3(output[0]));
                std::cout << "(output[0]:" << output[0];
            }
            else // intersection curve
            {
                easy3d::Graph::Vertex v1 = intersection_result->add_vertex(easy3d::vec3(output[0]));
                for (j = 1; j < output.size(); ++j)
                {
                    easy3d::Graph::Vertex v2 = intersection_result->add_vertex(easy3d::vec3(output[j]));
                    intersection_result->add_edge(v1, v2);
                    v1 = v2;
                }
            }
        }
        output.clear();
    }
    // set renderer
    intersection_result->set_name("intersection");
    singleintersection_result->set_name("single_intersection");
    add_model(intersection_result);
    add_model(singleintersection_result);
    mymod.singlepoint = singleintersection_result;
    mymod.insec = intersection_result;

    mymod.insec->renderer()->set_visible(true);
    mymod.insec->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
   // mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(false); //不显示点
    mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点
    mymod.singlepoint->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点

   // mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(0,0,0,1));//黑色
    mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
    mymod.insec->renderer()->get_lines_drawable("edges")->set_line_width(9.0f);
    mymod.insec->renderer()->get_lines_drawable("edges")->set_visible(false);// 显示点的连线

}

void MyViewer::TracePointintersection() {
    // surf
    NurbsObject* nurbsobj = dynamic_cast<NurbsObject*>(mymod.surf);

    if (!nurbsobj)
        return;
    int i;
    int j;
    int loopsize;
    //vector<std::vector<Point>>  pointout = NurbsPlaneOpenCurveInt(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf);
    vector<std::vector<Point>>  pointout = NurbsPlaneInt2(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf, 1);
    easy3d::Graph* intersection_result = new easy3d::Graph();
    easy3d::Graph* singleintersection_result = new easy3d::Graph();
    for (i = 0; i < pointout.size(); i++)
    {
        std::vector<easy3d::vec3> output; //假设这是你的输出
        for (const auto& p : pointout[i]) {
            output.emplace_back(p.x, p.y, p.z);
        }
        if (output.size() > 0)
        {
            if (output.size() == 1) //single point
            {

                singleintersection_result->add_vertex(easy3d::vec3(output[0]));
                std::cout << "(output[0]:" << output[0];
            }
            else // intersection curve
            {
                easy3d::Graph::Vertex v1 = intersection_result->add_vertex(easy3d::vec3(output[0]));
                for (j = 1; j < output.size(); ++j)
                {
                    easy3d::Graph::Vertex v2 = intersection_result->add_vertex(easy3d::vec3(output[j]));
                    intersection_result->add_edge(v1, v2);
                    v1 = v2;
                }
            }
        }
        output.clear();
    }
    // set renderer
    intersection_result->set_name("intersection");
    singleintersection_result->set_name("single_intersection");
    add_model(intersection_result);
    add_model(singleintersection_result);
    mymod.singlepoint = singleintersection_result;
    mymod.insec = intersection_result;

    mymod.insec->renderer()->set_visible(true);
    mymod.insec->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
   // mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(false); //不显示点
    mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点
    mymod.singlepoint->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点

   // mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(0,0,0,1));//黑色
    mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
    mymod.insec->renderer()->get_lines_drawable("edges")->set_line_width(9.0f);
    mymod.insec->renderer()->get_lines_drawable("edges")->set_visible(false);// 显示点的连线

}

void MyViewer::OpenCurveintersection() {
    // surf
    NurbsObject* nurbsobj = dynamic_cast<NurbsObject*>(mymod.surf);

    if (!nurbsobj)
        return;
    int i;
    int j;
    int loopsize;
    //vector<std::vector<Point>>  pointout = NurbsPlaneOpenCurveInt(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf);
    vector<std::vector<Point>>  pointout = NurbsPlaneInt2(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf, 2);
    easy3d::Graph* intersection_result = new easy3d::Graph();
    easy3d::Graph* singleintersection_result = new easy3d::Graph();
    for (i = 0; i < pointout.size(); i++)
    {
        std::vector<easy3d::vec3> output; //假设这是你的输出
        for (const auto& p : pointout[i]) {
            output.emplace_back(p.x, p.y, p.z);
        }
        if (output.size() > 0)
        {
            if (output.size() == 1) //single point
            {

                singleintersection_result->add_vertex(easy3d::vec3(output[0]));
                std::cout << "(output[0]:" << output[0];
            }
            else // intersection curve
            {
                easy3d::Graph::Vertex v1 = intersection_result->add_vertex(easy3d::vec3(output[0]));
                for (j = 1; j < output.size(); ++j)
                {
                    easy3d::Graph::Vertex v2 = intersection_result->add_vertex(easy3d::vec3(output[j]));
                    intersection_result->add_edge(v1, v2);
                    v1 = v2;
                }
            }
        }
        output.clear();
    }
    // set renderer
    intersection_result->set_name("intersection");
    singleintersection_result->set_name("single_intersection");
    add_model(intersection_result);
    add_model(singleintersection_result);
    mymod.singlepoint = singleintersection_result;
    mymod.insec = intersection_result;

    mymod.insec->renderer()->set_visible(true);
    mymod.insec->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
   // mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(false); //不显示点
    mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点
    mymod.singlepoint->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点

   // mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(0,0,0,1));//黑色
    mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
    mymod.insec->renderer()->get_lines_drawable("edges")->set_line_width(9.0f);
    mymod.insec->renderer()->get_lines_drawable("edges")->set_visible(false);// 显示点的连线

}

void MyViewer::Loopintersection() {
    // surf
    NurbsObject* nurbsobj = dynamic_cast<NurbsObject*>(mymod.surf);

    if (!nurbsobj)
        return;
    int i;
    int j;
    int loopsize;
   // vector<std::vector<Point>>  pointout = NurbsPlaneLoopInt(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf);
    vector<std::vector<Point>>  pointout = NurbsPlaneInt2(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf, 3);
    easy3d::Graph* intersection_result = new easy3d::Graph();
    easy3d::Graph* singleintersection_result = new easy3d::Graph();
    for (i = 0; i < pointout.size(); i++)
    {
        std::vector<easy3d::vec3> output; //假设这是你的输出
        for (const auto& p : pointout[i]) {
            output.emplace_back(p.x, p.y, p.z);
        }
        if (output.size() > 0)
        {
            if (output.size() == 1) //single point
            {

                singleintersection_result->add_vertex(easy3d::vec3(output[0]));
                std::cout << "(output[0]:" << output[0];
            }
            else // intersection curve
            {
                easy3d::Graph::Vertex v1 = intersection_result->add_vertex(easy3d::vec3(output[0]));
                for (j = 1; j < output.size(); ++j)
                {
                    easy3d::Graph::Vertex v2 = intersection_result->add_vertex(easy3d::vec3(output[j]));
                    intersection_result->add_edge(v1, v2);
                    v1 = v2;
                }
            }
        }
        output.clear();
    }
    // set renderer
    intersection_result->set_name("intersection");
    singleintersection_result->set_name("single_intersection");
    add_model(intersection_result);
    add_model(singleintersection_result);
    mymod.singlepoint = singleintersection_result;
    mymod.insec = intersection_result;

    mymod.insec->renderer()->set_visible(true);
    mymod.insec->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
   // mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(false); //不显示点
    mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点
    mymod.singlepoint->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点

   // mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(0,0,0,1));//黑色
    mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
    mymod.insec->renderer()->get_lines_drawable("edges")->set_line_width(9.0f);
    mymod.insec->renderer()->get_lines_drawable("edges")->set_visible(false);// 显示点的连线

}

void MyViewer::intersectionOCC()
{

    // surf
    NurbsObject* nurbsobj = dynamic_cast<NurbsObject*>(mymod.surf);

    if (!nurbsobj)
        return;
    //nurbsobj -> splineSurf;
    int i;
    int j;

    // ON_NurbsSurface nurbs_surf = *(casgeo::casBspline2opNurbs(nurbsobj->splineSurf));

     //vector<std::vector<Point>>  pointout = NurbsPlaneInt(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf);
     //////////////occ测试
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    vector<std::vector<Point>>  pointout;
    Point pointpush;
    std::vector<std::vector<double>> intersection_points;

    ON_NurbsSurface occPlane = CreatePlane(plane_point, plane_normal, 40);
    nurbsintersection_OCC(nurbsobj->splineSurf, occPlane, intersection_points);
    /*   for (size_t i = 0; i < intersection_points.size(); ++i) {
           std::cout << "i:" << i<< std::endl;
           int intsize = intersection_points[i].size();
           for (j = 0; j < intersection_points[i].size(); j++) {
               std::cout  << intersection_points[i][j]<<"，" ;
           }

       }*/
    std::vector<Point> line;


    for (i = 0; i < intersection_points.size(); i++) {
        for (j = 0; j < intersection_points[i].size() / 3; j++) {
            pointpush.x = intersection_points[i][3 * j + 0];
            pointpush.y = intersection_points[i][3 * j + 1];
            pointpush.z = intersection_points[i][3 * j + 2];
            line.push_back(pointpush);
        }
        pointout.push_back(line);
        line.clear();
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //vector<std::vector<Point>>  singlepointout;//输出单个点用于检测
    //Point Ptest = { 0,0,0 };
    //vector<Point> Pt = { Ptest };
    //singlepointout.push_back(Pt);
    easy3d::Graph* intersection_result = new easy3d::Graph();
    easy3d::Graph* singleintersection_result = new easy3d::Graph();
    for (i = 0; i < pointout.size(); i++) 
    {
        std::vector<easy3d::vec3> output; //假设这是你的输出
        for (const auto& p : pointout[i]) {
            output.emplace_back(p.x, p.y, p.z);
        }
        //for (i = 0; i < singlepointout.size(); i++) {
        //       std::vector<easy3d::vec3> output; //假设这是你的输出
        //       for (const auto& p : singlepointout[i]) {
        //           output.emplace_back(p.x, p.y, p.z);
        //       }

           //if (output.size() == 0) // no intersection
           //    return;

           // add points to graph
          // easy3d::Graph* intersection_result = new easy3d::Graph();
       
        if (output.size() > 0) 
        {
            if (output.size() == 1) {
                singleintersection_result->add_vertex(easy3d::vec3(output[0]));
            }
            else
            {
                easy3d::Graph::Vertex v1 = intersection_result->add_vertex(easy3d::vec3(output[0]));
                for (j = 1; j < output.size(); ++j)
                {
                    easy3d::Graph::Vertex v2 = intersection_result->add_vertex(easy3d::vec3(output[j]));
                    intersection_result->add_edge(v1, v2);
                    v1 = v2;
                }
            }
        }

        output.clear();
        // intersection_result->clear();
    }
    // set renderer
    intersection_result->set_name("intersection_occ");
    singleintersection_result->set_name("single_intersection_occ");
    add_model(intersection_result);
    add_model(singleintersection_result);
    mymod.singlepoint_occ = singleintersection_result;
    mymod.insec_occ = intersection_result;
    mymod.insec_occ->renderer()->set_visible(true);
    mymod.insec_occ->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(0, 0, 0, 1));//黑
    mymod.insec_occ->renderer()->get_points_drawable("vertices")->set_visible(false); //不显示点
    //mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点
    mymod.singlepoint_occ->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点

    mymod.insec_occ->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(0, 0, 0, 1));//黑色
    mymod.insec_occ->renderer()->get_lines_drawable("edges")->set_line_width(9.0f);
    // mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
    mymod.insec_occ->renderer()->get_lines_drawable("edges")->set_visible(true);// 显示点的连线


}
void MyViewer::intersectionparasolid()
{
   
    // 惰性启动：整个进程只启动一次
    static bool parasolid_started = false;
    if (!parasolid_started) {
        PK_SESSION_start_o_t start_options;
        PK_SESSION_start_o_m(start_options);
        PK_ERROR_code_t start_err = PK_SESSION_start(&start_options);
        if (start_err != PK_ERROR_no_errors) {
            std::cerr << "Parasolid start failed (error " << static_cast<int>(start_err)
                      << "). Check CASGEO_ENABLE_PARASOLID, PARASOLID_ROOT, and pskernel.dll." << std::endl;
            return;
        }
        parasolid_started = true;
    }
    // surf
    NurbsObject* nurbsobj = dynamic_cast<NurbsObject*>(mymod.surf);

    if (!nurbsobj)
        return;
    //nurbsobj -> splineSurf;
    int i;
    int j;

    // ON_NurbsSurface nurbs_surf = *(casgeo::casBspline2opNurbs(nurbsobj->splineSurf));

     //vector<std::vector<Point>>  pointout = NurbsPlaneInt(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf);
     //////////////occ测试
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    vector<std::vector<Point>>  pointout;
    Point pointpush;
    std::vector<std::vector<double>> intersection_points;

    ON_NurbsSurface occPlane = CreatePlane(plane_point, plane_normal, 40);
    nurbsintersection_Parasolid(nurbsobj->splineSurf, occPlane, intersection_points);
    /*   for (size_t i = 0; i < intersection_points.size(); ++i) {
           std::cout << "i:" << i<< std::endl;
           int intsize = intersection_points[i].size();
           for (j = 0; j < intersection_points[i].size(); j++) {
               std::cout  << intersection_points[i][j]<<"，" ;
           }

       }*/
    std::vector<Point> line;


    for (i = 0; i < intersection_points.size(); i++) {
        for (j = 0; j < intersection_points[i].size() / 3; j++) {
            pointpush.x = intersection_points[i][3 * j + 0];
            pointpush.y = intersection_points[i][3 * j + 1];
            pointpush.z = intersection_points[i][3 * j + 2];
            line.push_back(pointpush);
        }
        pointout.push_back(line);
        line.clear();
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //vector<std::vector<Point>>  singlepointout;//输出单个点用于检测
    //Point Ptest = { 0,0,0 };
    //vector<Point> Pt = { Ptest };
    //singlepointout.push_back(Pt);
    easy3d::Graph* intersection_result = new easy3d::Graph();
    easy3d::Graph* singleintersection_result = new easy3d::Graph();
    for (i = 0; i < pointout.size(); i++) 
    {
        std::vector<easy3d::vec3> output; //假设这是你的输出
        for (const auto& p : pointout[i]) {
            output.emplace_back(p.x, p.y, p.z);
        }
        //for (i = 0; i < singlepointout.size(); i++) {
        //       std::vector<easy3d::vec3> output; //假设这是你的输出
        //       for (const auto& p : singlepointout[i]) {
        //           output.emplace_back(p.x, p.y, p.z);
        //       }

           //if (output.size() == 0) // no intersection
           //    return;

           // add points to graph
          // easy3d::Graph* intersection_result = new easy3d::Graph();
       
        if (output.size() > 0) 
        {
            if (output.size() == 1) {
                singleintersection_result->add_vertex(easy3d::vec3(output[0]));
            }
            else
            {
                easy3d::Graph::Vertex v1 = intersection_result->add_vertex(easy3d::vec3(output[0]));
                for (j = 1; j < output.size(); ++j)
                {
                    easy3d::Graph::Vertex v2 = intersection_result->add_vertex(easy3d::vec3(output[j]));
                    intersection_result->add_edge(v1, v2);
                    v1 = v2;
                }
            }
        }

        output.clear();
        // intersection_result->clear();
    }
    // set renderer
    intersection_result->set_name("intersection_parasolid");
    singleintersection_result->set_name("single_intersection_parasolid");
    add_model(intersection_result);
    add_model(singleintersection_result);
    mymod.singlepoint_occ = singleintersection_result;
    mymod.insec_occ = intersection_result;
    mymod.insec_occ->renderer()->set_visible(true);
    mymod.insec_occ->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(0, 0, 0, 1));//黑
    mymod.insec_occ->renderer()->get_points_drawable("vertices")->set_visible(false); //不显示点
    //mymod.insec->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点
    mymod.singlepoint_occ->renderer()->get_points_drawable("vertices")->set_visible(true); //显示点

    mymod.insec_occ->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(0, 0, 0, 1));//黑色
    mymod.insec_occ->renderer()->get_lines_drawable("edges")->set_line_width(9.0f);
    // mymod.insec->renderer()->get_lines_drawable("edges")->set_uniform_coloring(easy3d::vec4(1, 0, 0, 1));//红
    mymod.insec_occ->renderer()->get_lines_drawable("edges")->set_visible(true);// 显示点的连线


}


void MyViewer::updatePlane()
{
    if (mymod.insec) { delete_model(mymod.insec); mymod.insec = nullptr; }
    if (mymod.singlepoint) { delete_model(mymod.singlepoint);  mymod.singlepoint = nullptr; }

    easy3d::vec3 p(plane_point[0], plane_point[1], plane_point[2]);
    easy3d::vec3 n(plane_normal[0], plane_normal[1], plane_normal[2]);

    // 1. 归一化法向量
    float len = n.norm();
    n =  n / len;

    if (mymod.plane) delete_model(mymod.plane);

    // 2. 选择一个参考向量，确保不与法向量平行
    easy3d::vec3 up_ref(0, 0, 1);
    if (std::abs(dot(up_ref, n)) > 0.99f) {
        up_ref = easy3d::vec3(0, 1, 0);
    }
    if (std::abs(dot(up_ref, n)) > 0.99f) {
        up_ref = easy3d::vec3(1, 0, 0);
    }

    // 3. 创建第一个基向量 u（与法向量垂直）
    easy3d::vec3 u = cross(up_ref, n);
    u = normalize(u);

    // 4. 创建第二个基向量 v（与 u 和 n 都垂直）
    easy3d::vec3 v = cross(n, u);  // 注意：n × u = -v，但我们用 u × n
    v = normalize(v);

    // 5. 计算矩形的四个顶点
  

    easy3d::vec3 p1 = p  -half_w * u - half_h * v; // 左下
    easy3d::vec3 p2 = p + half_w * u - half_h * v; // 右下
    easy3d::vec3 p3 = p + half_w * u + half_h * v; // 右上
    easy3d::vec3 p4 = p - half_w * u + half_h * v; // 左上

  
    mymod.plane = new SurfaceMesh();

    easy3d::SurfaceMesh::Vertex vert1 = mymod.plane->add_vertex(p1);
    easy3d::SurfaceMesh::Vertex vert2 = mymod.plane->add_vertex(p2);
    easy3d::SurfaceMesh::Vertex vert3 = mymod.plane->add_vertex(p3);
    easy3d::SurfaceMesh::Vertex vert4 = mymod.plane->add_vertex(p4);

    mymod.plane->add_triangle(vert1, vert2, vert3);
    mymod.plane->add_triangle(vert4, vert1, vert3);

    // set render
    mymod.plane->set_name("plane");
    add_model(mymod.plane);
    mymod.plane->renderer()->set_visible(drawplane);
    // 修改平面颜色
    auto plane_faces = mymod.plane->renderer()->get_triangles_drawable("faces");
    if (plane_faces) {
        //plane_faces->set_uniform_coloring(easy3d::vec4(255.0f / 255.0f, 255.0f / 255.0f, 255.0f / 255.0f, 0.5f)); // 纯白
        //plane_faces->state().set_back_color(easy3d::vec4(255.0f / 255.0f, 255.0f / 255.0f, 255.0f / 255.0f, 0.5f));
        //plane_faces->set_uniform_coloring(easy3d::vec4(220 / 255.0f, 210 / 255.0f, 190 / 255.0f, 1.0f));
        //plane_faces->state().set_back_color(easy3d::vec4(220 / 255.0f, 210 / 255.0f, 190 / 255.0f, 1.0f));//米黄色
        // 浅米色 - 比象牙白略深，保持暖调
       plane_faces->set_uniform_coloring(easy3d::vec4(240 / 255.0f, 235 / 255.0f, 220 / 255.0f, 0.0f));//算例颜色
       plane_faces->state().set_back_color(easy3d::vec4(240 / 255.0f, 235 / 255.0f, 220 / 255.0f, 0.0f));//算例颜色
    }

}

// menu
void MyViewer::draw_overlay(bool* visible)
{
    //ImGui::ShowDemoWindow();
    ImGui::SetNextWindowSize(ImVec2(300 * widget_scaling(), 200 * widget_scaling()), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(alpha_); // Transparent background
    if (ImGui::Begin("Model Operation", 0, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav))
    {
        ImGui::Text("Easy3DViewer");
        {
            if (ImGui::Checkbox("Draw surface", &drawsurf))
            {
                if (mymod.surf)
                    mymod.surf->renderer()->set_visible(drawsurf);
            }

            if (ImGui::Checkbox("Draw plane", &drawplane))
            {
                if (mymod.plane)
                    mymod.plane->renderer()->set_visible(drawplane);
            }

            if (ImGui::Checkbox("Draw control grid", &drawctrl))
            {
                if (mymod.ctrlnet)
                    mymod.ctrlnet->renderer()->set_visible(drawctrl);
            }

            if (ImGui::Checkbox("Draw control points", &drawctrlpts))
            {
                if (mymod.ctrlnet)
                    mymod.ctrlnet->renderer()->get_points_drawable("vertices")->set_visible(drawctrlpts);
            }

            if (ImGui::Checkbox("Draw intersection", &drawinsec))
            {
                if (mymod.insec)
                    mymod.insec->renderer()->set_visible(drawinsec);
                if (mymod.singlepoint)
                    mymod.singlepoint->renderer()->set_visible(drawinsec);
            }

            if (ImGui::Checkbox("Draw intersection occ", &drawinsec_occ))
            {
                if (mymod.insec_occ)
                    mymod.insec_occ->renderer()->set_visible(drawinsec_occ);
                if (mymod.singlepoint_occ)
                    mymod.singlepoint_occ->renderer()->set_visible(drawinsec_occ);
            }


            ImGui::Text("Plane Parameter");
            ImGui::DragFloat3("point", plane_point,0.01,-10,10);
            ImGui::DragFloat3("normal", plane_normal,0.1,-10,10);
         
            ImGui::DragFloat("planewide", &half_w, 0.5, 2, 20);
            ImGui::DragFloat("planelength", &half_h,0.5, 2, 20);
            if (ImGui::Button("Generate Plane"))
            {
                updatePlane();
            }

            if (ImGui::Button("NurbsPlaneInt"))
            {
               intersection();
            }
            if (ImGui::Button("NurbsPlaneIntOCC"))
            {
               intersectionOCC();
            }
            if (ImGui::Button("Parasolid"))
            {
                intersectionparasolid();
            }
            if (ImGui::Button("My Loop Trace Point"))
            {
                TracePointintersection();
            }
            if (ImGui::Button("My Open Curve"))
            {
               OpenCurveintersection();
            }
            if (ImGui::Button("My Loop"))
            {
                Loopintersection();


            }
            if (ImGui::Button("Map The TraceLine"))
            {
               // // surf
                NurbsObject* nurbsobj = dynamic_cast<NurbsObject*>(mymod.surf);

                if (!nurbsobj)
                    return;
                vector<std::vector<Point>> groups = NurbsPlaneInt2(plane_point[0], plane_point[1], plane_point[2], plane_normal, nurbsobj->splineSurf, 0);
                for (int i = 0; i < groups.size(); i++) {
                 int tg = groups[i].size();
                }
              
               int maptrace =  Maptraceline( groups);
               //
               /////////////////////////
               //////////////////
              //////////////occ交点投影
              /* vector<std::vector<Point>>  pointout;
               vector<Point> line;
               Point pointpush;
               std::vector<std::vector<double>> intersection_points;
               vector<std::vector<Point>> groups;
               ON_NurbsSurface occPlane = CreatePlane(plane_point, plane_normal, 40);
               nurbsintersection_OCC(nurbsobj->splineSurf, occPlane, intersection_points);

               for (int i = 0; i < intersection_points.size(); i++) {
                   for (int j = 0; j < intersection_points[i].size() / 3; j++) {
                       pointpush.x = intersection_points[i][3 * j + 0];
                       pointpush.y = intersection_points[i][3 * j + 1];
                       pointpush.z = intersection_points[i][3 * j + 2];
                       line.push_back(pointpush);
                   }
                   pointout.push_back(line);
                   line.clear();
               }
               int maptrace = Maptraceline(pointout);*/
            }

        }
    }
    ImGui::End();
}



void MyViewer::draw_menu_file()
{
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Open", "Ctrl+O"))
        {
            const std::string title("Please choose a file");
            const std::string& default_path = resource::directory() + "/../models/";
            const std::vector<std::string>& filters = {
                    "Bspline (*.nurbs)", "*.nurbs",
                    "Surface Mesh (*.obj *.ply *.off *.stl *.sm)", "*.obj *.ply *.off *.stl *.sm",
                    "Point Cloud (*.bin *.ply *.xyz *.bxyz *.las *.laz *.vg *.bvg *.ptx)",
                    "*.bin *.ply *.xyz *.bxyz *.las *.laz *.vg *.bvg *.ptx",
                    "Polytope Mesh (*.plm *.pm *.mesh)", "*.plm *.pm *.mesh",
                    "All Files (*.*)", "*"
            };
            const std::vector<std::string>& file_names = dialog::open(title, default_path, filters, true);

            if (file_names.size() > 0)
            {
                initmodel(file_names);
            }
        }
        if (ImGui::MenuItem("Save As...", "Ctrl+S"))
            save();


        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Alt+F4"))
            glfwSetWindowShouldClose(window_, GLFW_TRUE);

        ImGui::EndMenu();
    }
}

void MyViewer::draw_menu_view()
{
    if (ImGui::BeginMenu("View"))
    {
        if (ImGui::MenuItem("Snapshot", nullptr))
            snapshot();

        ImGui::Separator();
        if (ImGui::BeginMenu("Options"))
        {
            ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.50f);

            static int style_idx = 1;
            if (ImGui::Combo("Style", &style_idx, "Classic\0Dark\0Light\0"))
            {
                switch (style_idx)
                {
                case 0: ImGui::StyleColorsClassic(); break;
                case 1: ImGui::StyleColorsDark(); break;
                case 2: ImGui::StyleColorsLight(); break;
                }
            }

            ImGui::Checkbox("Panel Movable", &movable_);
            ImGui::ColorEdit3("Background Color", (float*)background_color_, ImGuiColorEditFlags_NoInputs);
            ImGui::DragFloat("Transparency", &alpha_, 0.005f, 0.0f, 1.0f, "%.1f");

            ImGui::PopItemWidth();
            ImGui::EndMenu();
        }

        ImGui::EndMenu();
    }
}

