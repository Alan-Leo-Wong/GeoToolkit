//
// Created by Lei on 2/12/2024.
//

#include "CGALVoronoi.hpp"
#include "../color/ColorMap.hpp"
#include <stack>
#include <iostream>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_2_to_lcc.h>
#include <CGAL/Triangulation_3_to_lcc.h>

NAMESPACE_BEGIN(GEOBOX)
    namespace core::voronoi {

        /////////////////////////////////////
        // ! Implementation of CGALVoronoi.
//        template<unsigned int DIM>
//        class CGALVoronoiImpl : CGALVoronoi {
//            typedef CGALVoronoiImpl<DIM> ThisClass;
//
//            struct _CGALVoronoiImpl{
//
//            };
//
//            typedef CGAL::Linear_cell_complex_for_combinatorial_map<DIM> LCC;
//            typedef CGAL::Delaunay_triangulation_3<LCC::Traits> Triangulation;
//
//            typedef LCC::Traits::Segment_3 Segment_3;
//            typedef LCC::Traits::Ray_3 Ray_3;
//            typedef LCC::Dart_descriptor Dart_3;
//            typedef LCC::Point Voronoi_Point3;
//            typedef LCC::Vertex_attribute_descriptor Vertex_attribute_descriptor_3;
//            typedef CGAL::Bbox_3 Bbox_3;
//        };


        void CGALVoronoi::remove_inf_cells(LCC_3 &alcc, Dart_3 adart) {
            // We remove the infinite volume plus all the volumes adjacent to it.
            // Indeed, we cannot view these volumes since they do not have
            // a "correct geometry".
            std::stack<Dart_3> toremove;
            LCC_3::size_type mark_toremove = alcc.get_new_mark();

            // adart belongs to the infinite volume.
            toremove.push(adart);
            CGAL::mark_cell<LCC_3, 3>(alcc, adart, mark_toremove);

            // Now we get all the volumes adjacent to the infinite volume.
            for (LCC_3::Dart_of_cell_range<3>::iterator
                         it = alcc.darts_of_cell<3>(adart).begin(),
                         itend = alcc.darts_of_cell<3>(adart).end(); it != itend; ++it) {
                if (!alcc.is_marked(alcc.beta(it, 3), mark_toremove)) {
                    CGAL::mark_cell<LCC_3, 3>(alcc, alcc.beta(it, 3), mark_toremove);
                    toremove.push(alcc.beta(it, 3));
                }
            }

            while (!toremove.empty()) {
                alcc.remove_cell<3>(toremove.top());
                toremove.pop();
            }

            assert(alcc.is_without_boundary(1) && alcc.is_without_boundary(2));

            /*std::cout << "Voronoi subdvision, only finite volumes:" << std::endl << "  ";
            alcc.display_characteristics(std::cout) << ", valid="
                                                    << alcc.is_valid()
                                                    << std::endl;*/
        }

        bool CGALVoronoi::write_obj(CGALVoronoi::LCC_3 &alcc, Dart_3 &dd, const char *filename) {
            std::ofstream out(filename);

            if (!alcc.are_all_faces_closed()) {
                std::cerr << "Impossible to write in obj a map having open faces." << std::endl;
                return false;
            }

            typedef typename LCC_3::Vertex_attribute_range::const_iterator VCI;
            VCI vit, vend = alcc.vertex_attributes().end();

            // TODO FOR index we do not need the Unique_hash_map.
            size_t i = 1;
            CGAL::Unique_hash_map<typename LCC_3::Vertex_attribute_const_descriptor,
                    size_t, typename LCC_3::Hash_function> index;
            for (vit = alcc.vertex_attributes().begin(); vit != vend; ++vit) {
                out << "v " << ::CGAL::to_double(vit->point().x()) << " " <<
                    ::CGAL::to_double(vit->point().y()) << " " <<
                    ::CGAL::to_double(vit->point().z()) << std::endl;
                index[vit] = i++; // TODO for index
            }

            typename LCC_3::size_type m = alcc.get_new_mark();

            remove_inf_cells(alcc, dd);

            // write finite volumes
            for (typename LCC_3::Dart_range::iterator itall = alcc.darts().begin(),
                         itallend = alcc.darts().end(); itall != itallend; ++itall) {
                if (!alcc.is_marked(itall, m)) {
                    std::size_t n = 0;
                    typename LCC_3::Dart_descriptor cur = itall;

                    CGAL_assertion(n >= 3);

                    //alcc.attribute<0>(alcc.dart_descriptor(*itall));

                    out << "f ";
                    // write the indices of vertices.
                    do {
                        out << index[alcc.vertex_attribute(cur)] << " ";
                        alcc.mark(cur, m);
                        alcc.mark(alcc.other_orientation(cur), m); // for GMap only, for CMap
                        CGAL_assertion(alcc.is_next_exist(cur));           // marks the same dart twice
                        cur = alcc.next(cur);
                    } while (cur != itall);

                    out << "\n";
                }
            }
            alcc.free_mark(m);

            for (auto &it: inf_cell_dual_attribute) {
                out << "l " << index[it.second.first] << " " << index[it.second.second] << std::endl;
            }

            out.close();
        }

        bool CGALVoronoi::write_obj_by_cell(LCC_3 &alcc, Dart_3 &dd, const char *filename) {
            std::ofstream out(filename);

            if (!alcc.are_all_faces_closed()) {
                std::cerr << "Impossible to write in obj a map having open faces." << std::endl;
                return false;
            }

            typedef typename LCC_3::Vertex_attribute_range::const_iterator VCI;
            VCI vit, vend = alcc.vertex_attributes().end();

            typename LCC_3::size_type mark = alcc.get_new_mark();

            remove_inf_cells(alcc, dd);

            int cell_idx = 0;
            int vert_idx = 0;

            using namespace color;
            ColorMap cm(ColorMap::INNER_CM::RAINBOW);

            for (LCC_3::One_dart_per_cell_range<3>::iterator it = alcc.one_dart_per_cell<3>().begin();
                 it != alcc.one_dart_per_cell<3>().end(); ++it) {
                std::queue<Dart_3> q;
                std::map<typename LCC_3::Vertex_attribute_const_descriptor,
                        size_t> cell_vert_index_map;
                q.push(it);
                while (!q.empty()) {
                    Dart_3 fro_dart = q.front();
                    q.pop();
                    if (alcc.is_marked(fro_dart, mark)) continue;

                    std::vector<size_t> face_vert_index;
                    Dart_3 cur_dart = fro_dart;
                    do {
                        alcc.mark(cur_dart, mark);

                        LCC_3::Vertex_attribute_const_descriptor vertex_attribute = alcc.vertex_attribute(cur_dart);
                        if (!cell_vert_index_map.count(vertex_attribute)) {
                            cell_vert_index_map[vertex_attribute] = ++vert_idx;
                            out << "v " << vertex_attribute->point()
                                << " " << cm.get_color(cell_idx % 256)().transpose()
                                // << " 0.492157 0.012320 0.999981" // TODO: vertex color
                                << std::endl;
                        }
                        face_vert_index.emplace_back(cell_vert_index_map.at(vertex_attribute));

                        q.push(alcc.beta<2>(cur_dart));

                        cur_dart = alcc.next(cur_dart);
                    } while (cur_dart != fro_dart);

                    out << "f";
                    for (size_t vert_index: face_vert_index)
                        out << " " << vert_index;
                    out << std::endl;
                }

                ++cell_idx;
            }
            alcc.unmark_all(mark);
            alcc.free_mark(mark);

            std::map<typename LCC_3::Vertex_attribute_const_descriptor,
                    size_t> inf_cell_vert_index_map;
            for (auto &it: inf_cell_dual_attribute) {
                if (!inf_cell_vert_index_map.count(it.second.first)) {
                    inf_cell_vert_index_map[it.second.first] = ++vert_idx;
                    out << "v " << it.second.first->point() << std::endl;
                }
                if (!inf_cell_vert_index_map.count(it.second.second)) {
                    inf_cell_vert_index_map[it.second.second] = ++vert_idx;
                    out << "v " << it.second.second->point() << std::endl;
                }

                out << "l " << inf_cell_vert_index_map.at(it.second.first) << " "
                    << inf_cell_vert_index_map.at(it.second.second) << std::endl;
            }

            out.close();

            return true;
        }

        void CGALVoronoi::vor2d() {

        }

        void CGALVoronoi::vor3d() {
            // 1) Generate Delaunay triangulation
            Triangulation_3 T;
            for (index_t i = 0; i < nb_sites; ++i) {
                Voronoi_Point3 p(
                        sites(i, 0),
                        sites(i, 1),
                        sites(i, 2)
                );
                T.insert(p);
            }
            if (!T.is_valid(false)) {
                // TODO: error message
                // TODO: voronoi wrapper pointer
                return;
            }

            // 2) Convert the triangulation into a 3D lcc.
            LCC_3 lcc;
            std::map<Triangulation_3::Cell_handle, Dart_3> vol_to_dart;
            Dart_3 d = CGAL::import_from_triangulation_3<LCC_3, Triangulation_3>(lcc, T, &vol_to_dart);

            std::cout << "Delaunay triangulation :" << std::endl << "  ";
            lcc.display_characteristics(std::cout) << ", valid="
                                                   << lcc.is_valid() << std::endl;

            // 3) Compute the dual lcc.
            LCC_3 dual_lcc;
            Dart_3 dd = lcc.dual(dual_lcc, d);
            // Here, dual_lcc is the 3D Voronoi diagram.
            assert(dual_lcc.is_without_boundary());

            // 4) Update the geometry of dual_lcc by using the std::map vol_to_dart.
            // vol_to_dart[vor_cell] = dart (The point of the dart is the circumcenter of tetrahedron)
            transform_dart_to_their_dual<LCC_3, Triangulation_3>
                    (lcc, dual_lcc, vol_to_dart);
            set_geometry_of_dual<LCC_3, Triangulation_3>(dual_lcc, T, vol_to_dart);

            // 5) TODO: Write to wrapper
            write_obj(dual_lcc, dd, "dual_lcc.obj");
        }

    }
NAMESPACE_END(GEOBOX)