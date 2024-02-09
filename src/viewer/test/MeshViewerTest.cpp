//
// Created by Lei on 2/9/2024.
//
#include "../MeshViewer.hpp"
#include <igl/read_triangle_mesh.h>
#include <string>

NAMESPACE_BEGIN(PROJ_NAME)
    namespace viewer::unit_test {
        using namespace detail;

        inline bool testMeshViewer(const std::string &modelFile) {
            MatrixX V;
            MatrixXi F;
            igl::read_triangle_mesh(modelFile, V, F);

            viewer::MeshViewer meshViewer;
            meshViewer.data().set_mesh(V, F);
            meshViewer.launch();

            return true;
        }

    } // namespace viewer::unit_test
NAMESPACE_END(PROJ_NAME)

int main(int argc, char **argv) {
    using namespace PROJ_NAME::viewer::unit_test;

    testMeshViewer(R"(F:\VisualStudioProgram\3DThinShell\model\bunny.off)");

    return 0;
}