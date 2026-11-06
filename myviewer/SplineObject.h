#pragma once

#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/graph.h>
#include "opennurbs.h"

class ControlNetObject : public easy3d::Graph 
{
public:
	ControlNetObject() :Graph() {
		name_ = "control_net";
	};
	virtual ~ControlNetObject() {};
private:
	bool isControlNet = true;
};

/**
 * @brief Nurbs viewer class.
 *
 * Visualize the nurbs surface and control net.
 */
class NurbsObject : public easy3d::SurfaceMesh 
{
public:
	ON_NurbsSurface splineSurf;
	ControlNetObject controlNet;

	NurbsObject(const ON_NurbsSurface& surface);
	virtual ~NurbsObject() {};

	void updateTriangulation();

private:
	std::vector<std::vector<easy3d::Graph::Vertex>> vList;
	int meshResolutionU = 400;//mesh resolution for triangulation
	int meshResolutionV = 400;//mesh resolution for triangulation
};

/**
 * @brief Read Nurbs Surfaces from file.
 *
 * @param file [in] file path.
 * @param capacity [in] maximum number of nurbs patches.
 * @param numNurbs [out] number of read nurbs patches.
 * @return NurbsObject** [out] nurbs patches pointers.
 */
NurbsObject* ReadNurbs(std::string file);

ON_NurbsSurface CreatePlane(float plane_point[3], float plane_normal[3], double size);