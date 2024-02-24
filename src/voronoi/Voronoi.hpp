//
// Created by Lei on 2/12/2024.
//

#ifndef GEOBOX_VORONOI_HPP
#define GEOBOX_VORONOI_HPP

#include <Config.hpp>
#include <BasicDataType.hpp>
#include <Geometry.hpp>

NAMESPACE_BEGIN(GEOBOX)
    namespace core::voronoi {
        using namespace detail;
        using namespace geometry;

        class PolyMesh;

        /**
         *
         */
        class Voronoi {
        public:
            Voronoi() = default;

            Voronoi(MatrixX _sites) : sites(std::move(_sites)) {
                nb_sites = sites.rows();
                sites_aabb = AABox<Vector3>(sites);
                sites_aabb.scaleAndTranslate(1.5);
            }

        public:
            /**
             *
             */
            virtual void vor2d() = 0;

            /**
             *
             */
            virtual void vor3d() = 0;

            /**
             *
             */
            virtual void rvd(const std::string& filename) = 0;

        public:
            MatrixX sites;
            index_t nb_sites;

            AABox<Vector3> sites_aabb;
        };

    }
NAMESPACE_END(GEOBOX)

#endif //GEOBOX_VORONOI_HPP
