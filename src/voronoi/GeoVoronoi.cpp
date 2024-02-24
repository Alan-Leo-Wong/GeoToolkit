//
// Created by Lei on 2/12/2024.
//

#include "GeoVoronoi.hpp"
#include "../color/ColorMap.hpp"
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/numerics/predicates.h>
#include <geogram/mesh/mesh.h>

NAMESPACE_BEGIN(GEOBOX)
    namespace core::voronoi {

        void GeoVoronoi::get_vor_cell(GEO::index_t v, GEO::ConvexCell &C) {
            GEO::PeriodicDelaunay3d::IncidentTetrahedra W_;
            delaunay_3->copy_Laguerre_cell_from_Delaunay(v, C, W_);

            if (!periodic_) {
                for (int i = 0; i < 6; ++i)
                    C.clip_by_plane(clip_planes[i]);
            }

            C.compute_geometry();
        }

        void GeoVoronoi::write_vor(const std::string &filename, double shrink, bool borders_only) {
            GEO::ConvexCell C;
            GEO::Mesh *mesh = new GEO::Mesh();

            std::vector<index_t> cell_vert_begin(nb_sites + 1, 0);

            using namespace color;
            ColorMap cm(ColorMap::INNER_CM::RAINBOW);
            std::vector<RGB> cell_color(nb_sites);

            for (index_t v = 0; v < nb_sites; ++v) {
                get_vor_cell(v, C);

                cell_color[v] = cm.get_random_color();

                // facet_attr 保存了 cell 的每个面中顶点的全局索引
                C.append_to_mesh(mesh, shrink, borders_only);

                cell_vert_begin[v] = mesh->vertices.nb();
            }

            int cell_idx = 0;
            std::ofstream out(filename);
            for (index_t v = 0; v < mesh->vertices.nb(); ++v) {
                if (cell_vert_begin[cell_idx] <= v) ++cell_idx;
                RGB vert_color = cell_color[cell_idx];

                const double *P = mesh->vertices.point_ptr(v);

                out << "v ";
                for (index_t c = 0; c < 3; ++c) {
                    out << P[c] << ' ';
                }
                out << vert_color << std::endl;

                for (index_t f: mesh->facets) {
                    out << "f ";
                    for (index_t c = mesh->facets.corners_begin(f);
                         c < mesh->facets.corners_end(f); ++c) {
                        out << mesh->facet_corners.vertex(c) + 1 << " ";
                    }
                    out << std::endl;
                }
            }
        }

        void GeoVoronoi::check_for_zero_area_facets(GEO::Mesh &M) {
            std::vector<GEO::index_t> remove_f;

            GEO::vec3 q1(0, 0, 0);
            GEO::vec3 q2(0, 0, 1);
            GEO::vec3 q3(0, 1, 0);
            GEO::vec3 q4(1, 0, 0);

            for (index_t f = 0; f < M.facets.nb(); ++f) {
                index_t c = M.facets.corners_begin(f);
                index_t v1 = M.facet_corners.vertex(c);
                index_t v2 = M.facet_corners.vertex(c + 1);
                index_t v3 = M.facet_corners.vertex(c + 2);
                const GEO::vec3 &p1 = GEO::Geom::mesh_vertex(M, v1);
                const GEO::vec3 &p2 = GEO::Geom::mesh_vertex(M, v2);
                const GEO::vec3 &p3 = GEO::Geom::mesh_vertex(M, v3);

                // Colinearity is tested by using four coplanarity
                // tests with points q1,q2,q3,q4 that are
                // not coplanar.
                if (
                        GEO::PCK::orient_3d(p1, p2, p3, q1) == 0 &&
                        GEO::PCK::orient_3d(p1, p2, p3, q2) == 0 &&
                        GEO::PCK::orient_3d(p1, p2, p3, q3) == 0 &&
                        GEO::PCK::orient_3d(p1, p2, p3, q4) == 0
                        ) {
                    GEO::Logger::warn("Validate") << "Found a zero-area facet"
                                                  << std::endl;
                    remove_f.resize(M.facets.nb(), 0);
                    remove_f[f] = 1;
                }
            }
            if (remove_f.size() != 0) {
                GEO::Logger::warn("Validate") << "Removing zero-area facet(s)"
                                              << std::endl;
                M.facets.delete_elements(remove_f);
            }
        }

        void GeoVoronoi::vor2d() {

        }

        void GeoVoronoi::vor3d() {
            delaunay_3 = new GEO::PeriodicDelaunay3d(periodic_, 1.0);
            if (!periodic_) {
                delaunay_3->set_keeps_infinite(true);
            }

            delaunay_3->set_vertices(nb_sites, points_.data());
            delaunay_3->compute();

            write_vor("geo_vor_3d.obj");
        }

        void GeoVoronoi::rvd(const std::string &mesh_file) {
            GEO::Mesh M_in;
            GEO::mesh_load(mesh_file, M_in);

            GEO::mesh_repair(M_in);
            check_for_zero_area_facets(M_in);

            rvd_delaunay = GEO::Delaunay::create(3);
            rvd_delaunay->set_vertices(
                    nb_sites, points_.data()
            );

            RVD = GEO::RestrictedVoronoiDiagram::create(
                    rvd_delaunay, &M_in
            );
            RVD->set_volumetric(false);

            GEO::Mesh M_out;
            RVD->compute_RVD(M_out, 0, false, false);

            std::string output_filename = "geo_rvd.obj";
            GEO::MeshIOFlags flags;
            // flags.set_attribute(GEO::MESH_FACET_REGION);
            // flags.set_attribute(GEO::MESH_CELL_REGION);
            // flags.set_element(GEO::MESH_CELLS);
            flags.set_element(GEO::MESH_FACETS);
            mesh_save(M_out, output_filename, flags);
        }

    }
NAMESPACE_END(GEOBOX)