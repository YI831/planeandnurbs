set(PROJECT_NAME NurbsPlaneInt)
project(${PROJECT_NAME})

add_library(${PROJECT_NAME}
	test.cpp
	test.h
        normal_cone.cpp
	normal_cone.h
        opennurbs3.cpp
    	)
 
target_include_directories(${PROJECT_NAME} PRIVATE ${EASY3D_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN_DIR})

################################################################################
###  OpenNURBS
################################################################################
target_include_directories (${PROJECT_NAME} PRIVATE ${EXTERNAL_LIBRARY_OPENNURBS_LIB_INCLUDE_DIR})
target_link_libraries (${PROJECT_NAME} PRIVATE ${EXTERNAL_LIBRARY_OPENNURBS_LIB_NAME})
###############################################################################
###################OCC
###############################################################
target_include_directories(${PROJECT_NAME} PRIVATE ${OCC_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE  $<$<CONFIG:Debug>:${OCC_DEBUG_DLL_LIBS}>
$<$<CONFIG:Release>:${OCC_DLL_LIBS}>)
######################################################################parasolid
if(CASGEO_ENABLE_PARASOLID)
    target_include_directories(${PROJECT_NAME}  PRIVATE ${PARASOLID_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME}  PRIVATE ${PARASOLID_PSKERNEL_LIBRARY})
endif()

################################################################################
###  SISL
################################################################################
target_include_directories(${PROJECT_NAME} PRIVATE ${SISL_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE sisl)