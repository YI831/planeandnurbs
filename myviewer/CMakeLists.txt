cmake_minimum_required(VERSION 3.12)

set(PROJECT_NAME demo)
project(${PROJECT_NAME})

add_executable(${PROJECT_NAME}
        main.cpp
        myviewer.cpp
        imviewer.cpp
	SplineObject.cpp
        myviewer.h
        imviewer.h
	SplineObject.h
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${EASY3D_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN_DIR})

target_link_libraries(${PROJECT_NAME} PRIVATE
        easy3d_core
        easy3d_renderer
        easy3d_viewer
        easy3d_gui
	easy3d_util
	easy3d_fileio
        3rd_imgui
	3rd_easyloggingpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE NurbsPlaneInt)

################################################################################
###  OpenNURBS
################################################################################
target_include_directories (${PROJECT_NAME} PRIVATE ${EXTERNAL_LIBRARY_OPENNURBS_LIB_INCLUDE_DIR})
target_link_libraries (${PROJECT_NAME} PRIVATE ${EXTERNAL_LIBRARY_OPENNURBS_LIB_NAME})
######################################################################parasolid
if(CASGEO_ENABLE_PARASOLID)
    target_include_directories(${PROJECT_NAME}  PRIVATE ${PARASOLID_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME}  PRIVATE ${PARASOLID_PSKERNEL_LIBRARY})
endif()

################################################################################
