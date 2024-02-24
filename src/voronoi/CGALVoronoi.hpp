//
// Created by Lei on 2/12/2024.
//

#ifndef GEOBOX_CGALVORONOI_HPP
#define GEOBOX_CGALVORONOI_HPP

#include "Voronoi.hpp"
#include <CGAL/Linear_cell_complex_for_combinatorial_map.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>

#include <utility>

NAMESPACE_BEGIN(GEOBOX)
    namespace core::voronoi {

        class CGALVoronoi : public Voronoi {
        public:
            ////////////////////////////////
            // ! Member type
            /*typedef CGAL::Exact_predicates_inexact_constructions_kernel Voronoi_K;
            typedef Voronoi_K::Point_2 Voronoi_Point2;
            typedef Voronoi_K::Point_3 Voronoi_Point3;*/
            /// 2D
            typedef CGAL::Linear_cell_complex_for_combinatorial_map<2> LCC_2;
            typedef LCC_2::Dart_descriptor Dart_2;
            typedef LCC_2::Point Voronoi_Point2;
            typedef CGAL::Delaunay_triangulation_2<LCC_2::Traits> Triangulation_2;
            /// 3D
            typedef CGAL::Linear_cell_complex_for_combinatorial_map<3> LCC_3;
            typedef CGAL::Delaunay_triangulation_3<LCC_3::Traits> Triangulation_3;
            typedef LCC_3::Traits::Segment_3 Segment_3;
            typedef LCC_3::Traits::Ray_3 Ray_3;
            typedef LCC_3::Dart_descriptor Dart_3;
            typedef LCC_3::Point Voronoi_Point3;
            typedef LCC_3::Vertex_attribute_descriptor Vertex_attribute_descriptor_3;
            typedef CGAL::Bbox_3 Bbox_3;

            ////////////////////////////////
            // ! Constructor and Destructor
            CGALVoronoi() = default;

            explicit CGALVoronoi(MatrixX _sites) : sites(std::move(_sites)) {
                // TODO: dimension check
            }

            ////////////////////////////////
            // ! Core of algorithm
            /**
             *
             * @tparam LCC
             * @tparam TR
             * @param alcc
             * @param adual
             * @param assoc
             */
            template<typename LCC, typename TR>
            void transform_dart_to_their_dual(LCC &alcc, LCC &adual,
                                              std::map<typename TR::Cell_handle,
                                                      typename LCC::Dart_descriptor> &assoc) {
                typename LCC::Dart_range::iterator it1 = alcc.darts().begin();
                typename LCC::Dart_range::iterator it2 = adual.darts().begin();

                std::map<typename LCC::Dart_descriptor, typename LCC::Dart_descriptor> dual;

                for (; it1 != alcc.darts().end(); ++it1, ++it2) {
                    dual[it1] = it2;
                }

                for (typename std::map<typename TR::Cell_handle, typename LCC::Dart_descriptor>
                ::iterator it = assoc.begin(), itend = assoc.end(); it != itend; ++it) {
                    assoc[it->first] = dual[it->second];
                }
            }

            /**
             *
             * @tparam LCC
             * @tparam TR
             * @param alcc
             * @param tr
             * @param assoc
             */
            template<typename LCC, typename TR>
            void set_geometry_of_dual(LCC &alcc, TR &tr,
                                      std::map<typename TR::Cell_handle,
                                              typename LCC::Dart_descriptor> &assoc) {

                std::map<Triangulation_3::Cell_handle, Vertex_attribute_descriptor_3> cell_vertex_attribute;

                for (typename std::map<typename TR::Cell_handle, typename LCC::Dart_descriptor>
                ::iterator it = assoc.begin(), itend = assoc.end(); it != itend; ++it) {
                    // set_vertex_attribute 将包含 0-dart(it->second) 的 dart 建立一个 0-attribute
                    if (!tr.is_infinite(it->first)) {
                        Vertex_attribute_descriptor_3 attribute = alcc.create_vertex_attribute(tr.dual(it->first));
                        alcc.set_vertex_attribute
                                (it->second, attribute); // tr 为 Dealunay triangle，其 cell 的对偶就是 voronoi 顶点
                        cell_vertex_attribute[it->first] = attribute;
                    } else {
                        Vertex_attribute_descriptor_3 attribute = alcc.create_vertex_attribute(
                                inf_cell_dual_point[it->first].target());
                        alcc.set_vertex_attribute(it->second, attribute);
                        cell_vertex_attribute[it->first] = attribute;
                    }
                }

                for (typename std::map<typename TR::Cell_handle, typename LCC::Dart_descriptor>
                ::iterator it = assoc.begin(), itend = assoc.end(); it != itend; ++it) {
                    if (tr.is_infinite(it->first)) {
                        for (int i = 0; i < 4; ++i) {
                            typename TR::Cell_handle cell_neighbor = it->first->neighbor(i);
                            if (!tr.is_infinite(cell_neighbor)) {
                                inf_cell_dual_attribute[it->first] =
                                        std::make_pair(cell_vertex_attribute[cell_neighbor],
                                                       cell_vertex_attribute[it->first]);
                            }
                        }
                    }
                }
            }

            /**
             * Function used to remove infinite cells of voronoi diagram.
             * @param alcc
             * @param adart
             */
            void remove_inf_cells(LCC_3 &alcc, Dart_3 adart);

            bool write_obj(LCC_3& alcc, Dart_3& dd, const char* filename);

            bool write_obj_by_cell(LCC_3& alcc, Dart_3& dd, const char* filename);

            /**
             *
             */
            void vor2d() override;

            /**
             *
             */
            void vor3d() override;

        private:
            std::map<Triangulation_3::Cell_handle, Segment_3> inf_cell_dual_point;

            std::map<Triangulation_3::Cell_handle,
                    std::pair<Vertex_attribute_descriptor_3, Vertex_attribute_descriptor_3>>
                    inf_cell_dual_attribute;
        };

    }
NAMESPACE_END(GEOBOX)

#endif //GEOBOX_CGALVORONOI_HPP
