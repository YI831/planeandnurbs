#pragma once

#include <easy3d/viewer/viewer.h>

#include <easy3d/core/surface_mesh.h>
#include <easy3d/algo/surface_mesh_sampler.h>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/texture_manager.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/resources.h>
#include <easy3d/util/logging.h>
#include <easy3d/renderer/shader_manager.h>
#include <easy3d/gui/picker_surface_mesh.h>

#include <3rd_party/glfw/include/GLFW/glfw3.h>    
#include <easy3d/renderer/shapes.h>

#include "imviewer.h"


struct ImGuiContext;
class PrimitiveShape;
#ifdef __APPLE__
#define EASY3D_MOD_CONTROL GLFW_MOD_SUPER
#else
#define EASY3D_MOD_CONTROL GLFW_MOD_CONTROL
#endif


class MyViewer : public easy3d::ViewerImGui
{
public:
	MyViewer(
		const std::string& title = "Easy3D ImGui Viewer",
		int samples = 4,
		int gl_major = 3,
		int gl_minor = 2,
		bool full_screen = false,
		bool resizable = true,
		int depth_bits = 24,
		int stencil_bits = 8
	);

protected:


	bool key_press_event(int key, int modifiers) override;

	bool mouse_press_event(int x, int y, int button, int modifiers)override;

	// Mouse button release event handler
	bool mouse_release_event(int x, int y, int button, int modifiers)override;
	/// Mouse drag (i.e., a mouse button was pressed) event handler
	bool mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers)override;

	virtual void draw_menu_file()override;
	virtual void draw_menu_view()override;
	virtual void draw_overlay(bool* visible)override;


public:

	void initrenderer();
	void initmodel(const std::vector<std::string>& files);
	void intersection();
	void intersectionOCC();
	void updatePlane();
	void Loopintersection();
	void TracePointintersection();
	void OpenCurveintersection();
	void intersectionparasolid();
};


