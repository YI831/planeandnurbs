#include "myviewer.h"
#include "easy3d/util/logging.h"
//#include "NurbsPlaneInt/test.h"
using namespace easy3d;

int main(int argc, char **argv) {
    // Initialize logging.
    easy3d::logging::initialize(true);
   
       
   
    // Create the default Easy3D viewer.
    // Note: a viewer must be created before creating any drawables.
    MyViewer viewer("PlaneNURBS_Int");

    // Load a mesh model to the viewer

   // viewer.initmodel(file_name);
    //viewer.current_model()->points();
    std::cout << "............."  << "............";
    // Run the viewer
   
    return viewer.run();
}
