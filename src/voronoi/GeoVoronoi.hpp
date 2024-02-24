//
// Created by Lei on 2/12/2024.
//

#ifndef GEOBOX_GEOVORONOI_HPP
#define GEOBOX_GEOVORONOI_HPP

#include "Voronoi.hpp"
#include "../utils/Common.hpp"
#include <geogram/delaunay/periodic_delaunay_3d.h>
#include <geogram/voronoi/RVD.h>

NAMESPACE_BEGIN(GEOBOX)
    namespace core::voronoi {

        class GeoVoronoi : public Voronoi {

            GeoVoronoi() {
                utils::matrix2vec(sites, points_);

                Scalar min_x = sites_aabb.boxOrigin.x();
                Scalar min_y = sites_aabb.boxOrigin.y();
                Scalar min_z = sites_aabb.boxOrigin.z();

                Scalar max_x = sites_aabb.boxEnd.x();
                Scalar max_y = sites_aabb.boxEnd.y();
                Scalar max_z = sites_aabb.boxEnd.z();

                clip_planes.resize(6);
                clip_planes[0] = GEO::vec4(1.0, 0, 0, min_x);
                clip_planes[1] = GEO::vec4(-1.0, 0, 0, max_x);

                clip_planes[2] = GEO::vec4(0, 1.0, 0, min_y);
                clip_planes[3] = GEO::vec4(0, -1.0, 0, max_y);

                clip_planes[4] = GEO::vec4(0, 0, 1.0, min_z);
                clip_planes[5] = GEO::vec4(0, 0, -1.0, max_z);
            }

            void vor2d() override;

            void get_vor_cell(GEO::index_t v, GEO::ConvexCell &C);

            void write_vor(const std::string& filename,
                           double shrink = 0.0, bool borders_only = false);

            void vor3d() override;

            void check_for_zero_area_facets(GEO::Mesh& M);

            void rvd(const std::string& filename) override;

        private:
            std::vector<double> points_;

            std::vector<GEO::vec4> clip_planes;

            bool periodic_ = false;

            GEO::SmartPointer<GEO::PeriodicDelaunay3d> delaunay_3;

            GEO::Delaunay_var rvd_delaunay;
            GEO::RestrictedVoronoiDiagram_var RVD;
        };

    }
NAMESPACE_END(GEOBOX)

#endif //GEOBOX_GEOVORONOI_HPP
