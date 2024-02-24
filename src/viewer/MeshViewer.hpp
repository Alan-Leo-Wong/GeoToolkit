//
// Created by Lei on 2/9/2024.
//
#ifndef GEOBOX_MESHVIEWER_HPP
#define GEOBOX_MESHVIEWER_HPP

#include <Config.hpp>
#include <detail/BasicDataType.hpp>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

NAMESPACE_BEGIN(GEOBOX)
namespace viewer {
    using namespace igl::opengl::glfw;

    class MeshViewer : public Viewer {
    private:
        igl::opengl::glfw::imgui::ImGuiPlugin plugin;
        igl::opengl::glfw::imgui::ImGuiMenu menu;

    public:
        MeshViewer() {
            // attach a menu plugin
            this->plugins.emplace_back(&plugin);
            plugin.widgets.emplace_back(&menu);
        }
    };

}
NAMESPACE_END(GEOBOX)

#endif //GEOBOX_MESHVIEWER_HPP
