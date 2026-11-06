cmake_minimum_required(VERSION 3.12)
message(STATUS "=== THIS IS THE ROOT CMakeLists.txt ===")

if (APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.12" CACHE STRING "Minimum OS X deployment version" FORCE)
    message(STATUS "Minimum OS X deployment version: ${CMAKE_OSX_DEPLOYMENT_TARGET}")
endif ()

################################################################################

project(Easy3DViewer)

set(EASY3D_MAJOR_VERSION 2)
set(EASY3D_MINOR_VERSION 4)
set(EASY3D_PATCH_VERSION 9)
set(EASY3D_VERSION "${EASY3D_MAJOR_VERSION}.${EASY3D_MINOR_VERSION}.${EASY3D_PATCH_VERSION}")

################################################################################

# Discourage users to build Easy3D directly in its root directory
if (${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    if (NOT SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED)
        message(WARNING "It is NOT advised to build Easy3D directly in its root directory!")
        set(SAME_BUILD_AND_SOURCE_FOLDER_WARNING_ALREADY_ISSUED TRUE)
    endif ()
endif ()

################################################################################

#math(EXPR EASY3D_OS_BITS "8 * ${CMAKE_SIZEOF_VOID_P}")
#message(STATUS "Easy3d: I'am woking on ${EASY3D_OS_BITS}-bit ${CMAKE_SYSTEM_NAME}")
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(STATUS "Easy3D: I am working on a 64-bit ${CMAKE_SYSTEM_NAME}")
    add_definitions(-DENV_64_BIT)
else ()
    message(STATUS "Easy3D: I am working on a 32-bit ${CMAKE_SYSTEM_NAME}")
    add_definitions(-DENV_32_BIT)
endif ()

################################################################################

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

################################################################################

# Detects whether this is a top-level project
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(EASY3D_TOPLEVEL_PROJECT ON)
else ()
    set(EASY3D_TOPLEVEL_PROJECT OFF)
endif ()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

################################################################################

# Build tutorials
option(EASY3D_BUILD_TUTORIALS       "Build Easy3D Tutorials"                                OFF)
# Build advanced examples/applications that require Qt (>= v5.6)
option(EASY3D_ENABLE_QT             "Build advanced examples/applications that require Qt (>= v5.6)"    OFF)
# Build advanced features that require CGAL (>= v5.1)
option(EASY3D_ENABLE_CGAL           "Build advanced features that require CGAL (>= v5.1)"               OFF)
# Build advanced features that require OCC
option(CASGEO_ENABLE_OCC           "Build advanced features that require OCC"               ON)
# Build features that require Parasolid
option(CASGEO_ENABLE_PARASOLID    "Build advanced features that require Parasolid"        ON)
################################################################################

if (EASY3D_ENABLE_CGAL)
    find_package(CGAL QUIET)
    if (CGAL_FOUND)
        if (CGAL_VERSION VERSION_GREATER_EQUAL "5.1")
            set(EASY3D_HAS_CGAL TRUE)
            add_definitions(-DHAS_CGAL)
            message(STATUS "Found CGAL v${CGAL_VERSION}")
        else ()
            set(EASY3D_HAS_CGAL FALSE)
            message(FATAL_ERROR "You have requested CGAL support and CGAL v${CGAL_VERSION} is found. However, Easy3D"
                    " requires at least v5.1. Please install CGAL v5.1 or later. In case you have multiple versions"
                    " of CGAL on your platform, provide the path of a suitable one to the CMake variable `CGAL_DIR`.\n"
                    " Please note that only the following surface mesh algorithms require CGAL (v5.1 or later): \n"
                    "   - reorientation; \n"
                    "   - detecting/resolving duplicate vertices/faces; \n"
                    "   - detecting/resolving self-intersection; \n"
                    "   - clipping/splitting/slicing. \n"
                    " If you don't need these algorithms, disable CGAL support by setting `EASY3D_ENABLE_CGAL` to `OFF`.\n"
                    " You will still be able to use all other features provided by Easy3D without CGAL.")
            return()
        endif ()
    else ()
        set(EASY3D_HAS_CGAL FALSE)
        message(FATAL_ERROR "You have requested CGAL support but CGAL (v5.1 or later required) was not found. You can"
                " set 'CGAL_DIR' to the directory containing `CGALConfig.cmake` or `cgal-config.cmake` to help CMake find CGAL.\n"
                " Please note that only the following surface mesh algorithms require CGAL (v5.1 or later): \n"
                "   - reorientation; \n"
                "   - detecting/resolving duplicate vertices/faces; \n"
                "   - detecting/resolving self-intersection; \n"
                "   - clipping/splitting/slicing. \n"
                " If you don't need these algorithms, disable CGAL support by setting `EASY3D_ENABLE_CGAL` to `OFF`.\n"
                " You will still be able to use all other features provided by Easy3D without CGAL.")
        return()
    endif ()
endif ()

if(CASGEO_ENABLE_OCC)
    message(INFO: "Program link Opencascade ON")
    add_definitions(-DCASGEO_WITH_OCC)
    
    if(WIN32)
        set(USER_OCC_ROOT D:/project/OpenCASCADE-7.7.0-vc14-64/opencascade-7.7.0 CACHE STRING "OpenCASCADE root dir")
        message(INFO: "Opencascade dir:" ${USER_OCC_ROOT})
        set(OCC_INCLUDE_DIR ${USER_OCC_ROOT}/inc CACHE STRING "OpenCASCADE include dir")

        set(OCC_LINK_DIR ${USER_OCC_ROOT}/win64/vc14/lib CACHE STRING "OpenCASCADE lib dir")
        file(GLOB OCC_DLL_LIBS "${OCC_LINK_DIR}/*.lib")
        message(INFO: "Opencascade libs:" ${OCC_DLL_LIBS})
        link_directories(${OCC_LINK_DIR})

        set(OCC_DEBUG_LINK_DIR ${USER_OCC_ROOT}/win64/vc14/libd CACHE STRING "OpenCASCADE lib dir")
        file(GLOB OCC_DEBUG_DLL_LIBS "${OCC_DEBUG_LINK_DIR}/*.lib")
    endif(WIN32)
endif(CASGEO_ENABLE_OCC)

# ============================================================
# Parasolid solver
# ============================================================
if(CASGEO_ENABLE_PARASOLID)
    if(NOT PARASOLID_ROOT)
        if(DEFINED ENV{PARASOLID_ROOT})
            file(TO_CMAKE_PATH "$ENV{PARASOLID_ROOT}" PARASOLID_ROOT)
        else()
            set(PARASOLID_ROOT "D:/project/ParaSolid-Local/EnvX64")
        endif()
    endif()

    set(PARASOLID_ROOT "${PARASOLID_ROOT}" CACHE PATH "Parasolid installation directory")
    set(PARASOLID_INCLUDE_DIR "${PARASOLID_ROOT}" CACHE PATH "Parasolid include directory")
    set(PARASOLID_PSKERNEL_LIBRARY "${PARASOLID_ROOT}/dll/pskernel.lib" CACHE FILEPATH "Parasolid kernel import library")
    set(PARASOLID_DLL "${PARASOLID_ROOT}/dll/pskernel.dll" CACHE FILEPATH "Parasolid kernel runtime library")

    foreach(required_file IN ITEMS
            "${PARASOLID_INCLUDE_DIR}/parasolid_kernel.h"
            "${PARASOLID_PSKERNEL_LIBRARY}"
            "${PARASOLID_DLL}")
        if(NOT EXISTS "${required_file}")
            message(FATAL_ERROR
                "Parasolid support is enabled, but a required file is missing: ${required_file}
"
                "Set PARASOLID_ROOT (or the corresponding cached file path) to the correct Parasolid installation.")
        endif()
    endforeach()

    if(NOT TARGET parasolid_kernel)
        add_library(parasolid_kernel SHARED IMPORTED GLOBAL)
        set_target_properties(parasolid_kernel PROPERTIES
            IMPORTED_IMPLIB "${PARASOLID_PSKERNEL_LIBRARY}"
            IMPORTED_LOCATION "${PARASOLID_DLL}"
            INTERFACE_INCLUDE_DIRECTORIES "${PARASOLID_INCLUDE_DIR}")
    endif()

    message(STATUS "Parasolid support: ON (${PARASOLID_ROOT})")
else()
    message(STATUS "Parasolid support: OFF")
endif()

function(casgeo_copy_parasolid_runtime target)
    if(CASGEO_ENABLE_PARASOLID AND WIN32)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${PARASOLID_DLL}"
                    "$<TARGET_FILE_DIR:${target}>"
            VERBATIM)
    endif()
endfunction()


################################################################################

### Configuration
set(EASY3D_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(EASY3D_THIRD_PARTY ${EASY3D_ROOT}/3rd_party)

set(EASY3D_INCLUDE_DIR ${EASY3D_ROOT})
set(EASY3D_SOURCE_DIR ${EASY3D_ROOT})
set(EASY3D_BINARY_DIR ${CMAKE_BINARY_DIR})

### conditionally compile certain modules depending on libraries found on the system
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

set(EIGEN_DIR ${CMAKE_CURRENT_LIST_DIR}/3rd_party/eigen)
set(SISL_INCLUDE_DIR ${EASY3D_ROOT}/3rd_party/SISL/include)

################################################################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

################################################################################

add_subdirectory(3rd_party)

add_subdirectory(easy3d)

if (EASY3D_BUILD_TUTORIALS)
    add_subdirectory(tutorials)
endif ()

add_subdirectory(myviewer)
add_subdirectory(NurbsPlaneInt)


################################################################################

# hide some variables that might be set in 3rd_party libraries
mark_as_advanced(FORCE BUILD_SHARED_LIBS)

################################################################################
