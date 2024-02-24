//
// Created by Lei on 2/21/2024.
//

#ifndef GEOBOX_GEOMETRY_HPP
#define GEOBOX_GEOMETRY_HPP

#include <Config.hpp>
#include <BasicDataType.hpp>
#include <string>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <random>
#include <omp.h>

NAMESPACE_BEGIN(GEOBOX)
    using namespace detail;

    namespace geometry::gvis {
        // Helper function to write single vertex to OBJ file
        static void write_vertex(std::ofstream &output, const Vector3 &v) {
            output << "v " << v.x() << " " << v.y() << " " << v.z() << std::endl;
        }

        // Helper function to write single vertex to OBJ file
        static void write_vertex(std::ofstream &output, const Vector3 &v, const Vector3 &rgb) {
            output << "v " << v.x() << " " << v.y() << " " << v.z() << " " << rgb.x() << " " << rgb.y() << " "
                   << rgb.z() << std::endl;
        }

        // Helper function to write single vertex to OBJ file
        static void write_vertex_to_xyz(std::ofstream &output, const Vector3 &v) {
            output << v.x() << " " << v.y() << " " << v.z() << std::endl;
        }

        // Helper function to write face
        static void write_face(std::ofstream &output, const Vector3i &f) {
            output << "f " << f.x() << " " << f.y() << " " << f.z() << std::endl;
        }

        // Helper function to write face
        static void write_face(std::ofstream &output, const Eigen::Vector4i &f) {
            output << "f " << f.x() << " " << f.y() << " " << f.z() << " " << f.w() << std::endl;
        }

        // Helper function to write line
        static void write_line(std::ofstream &output, const Eigen::Vector2i &l) {
            output << "l " << l.x() << " " << l.y() << std::endl;
        }

        // Helper function to write full cube (using relative vertex positions in the OBJ file - support for this should be widespread by now)
        inline void
        writeCube(const Vector3 &nodeOrigin, const Vector3 &unit, const size_t &faceBegIdx, std::ofstream &output) {
            //	   2-------1
            //	  /|      /|
            //	 / |     / |
            //	7--|----8  |
            //	|  4----|--3
            //	| /     | /
            //	5-------6
            // Create vertices
            Vector3 v1 = nodeOrigin + Vector3(0, unit.y(), unit.z());
            Vector3 v2 = nodeOrigin + Vector3(0, 0, unit.z());
            Vector3 v3 = nodeOrigin + Vector3(0, unit.y(), 0);
            const Vector3 &v4 = nodeOrigin;
            Vector3 v5 = nodeOrigin + Vector3(unit.x(), 0, 0);
            Vector3 v6 = nodeOrigin + Vector3(unit.x(), unit.y(), 0);
            Vector3 v7 = nodeOrigin + Vector3(unit.x(), 0, unit.z());
            Vector3 v8 = nodeOrigin + Vector3(unit.x(), unit.y(), unit.z());

            // write them in reverse order, so relative position is -i for v_i
            write_vertex(output, v1);
            write_vertex(output, v2);
            write_vertex(output, v3);
            write_vertex(output, v4);
            write_vertex(output, v5);
            write_vertex(output, v6);
            write_vertex(output, v7);
            write_vertex(output, v8);

            // create faces
#if defined(MESH_WRITE)
            // back
            write_face(output, Eigen::Vector3i(faceBegIdx + 1, faceBegIdx + 3, faceBegIdx + 4));
            write_face(output, Eigen::Vector3i(faceBegIdx + 1, faceBegIdx + 4, faceBegIdx + 2));
            // bottom
            write_face(output, Eigen::Vector3i(faceBegIdx + 4, faceBegIdx + 3, faceBegIdx + 6));
            write_face(output, Eigen::Vector3i(faceBegIdx + 4, faceBegIdx + 6, faceBegIdx + 5));
            // right
            write_face(output, Eigen::Vector3i(faceBegIdx + 3, faceBegIdx + 1, faceBegIdx + 8));
            write_face(output, Eigen::Vector3i(faceBegIdx + 3, faceBegIdx + 8, faceBegIdx + 6));
            // top
            write_face(output, Eigen::Vector3i(faceBegIdx + 1, faceBegIdx + 2, faceBegIdx + 7));
            write_face(output, Eigen::Vector3i(faceBegIdx + 1, faceBegIdx + 7, faceBegIdx + 8));
            // left
            write_face(output, Eigen::Vector3i(faceBegIdx + 2, faceBegIdx + 4, faceBegIdx + 5));
            write_face(output, Eigen::Vector3i(faceBegIdx + 2, faceBegIdx + 5, faceBegIdx + 7));
            // front
            write_face(output, Eigen::Vector3i(faceBegIdx + 5, faceBegIdx + 6, faceBegIdx + 8));
            write_face(output, Eigen::Vector3i(faceBegIdx + 5, faceBegIdx + 8, faceBegIdx + 7));
#elif defined(CUBE_WRITE)
            // back
            write_face(output, Eigen::Vector4i(faceBegIdx + 3, faceBegIdx + 4, faceBegIdx + 2, faceBegIdx + 1));
            // bottom
            write_face(output, Eigen::Vector4i(faceBegIdx + 6, faceBegIdx + 5, faceBegIdx + 4, faceBegIdx + 3));
            // right
            write_face(output, Eigen::Vector4i(faceBegIdx + 1, faceBegIdx + 8, faceBegIdx + 6, faceBegIdx + 3));
            // top
            write_face(output, Eigen::Vector4i(faceBegIdx + 1, faceBegIdx + 2, faceBegIdx + 7, faceBegIdx + 8));
            // left
            write_face(output, Eigen::Vector4i(faceBegIdx + 4, faceBegIdx + 5, faceBegIdx + 7, faceBegIdx + 2));
            // front
            write_face(output, Eigen::Vector4i(faceBegIdx + 8, faceBegIdx + 7, faceBegIdx + 5, faceBegIdx + 6));
#else
            write_line(output, Eigen::Vector2i(faceBegIdx + 1, faceBegIdx + 2));
            write_line(output, Eigen::Vector2i(faceBegIdx + 2, faceBegIdx + 7));
            write_line(output, Eigen::Vector2i(faceBegIdx + 7, faceBegIdx + 8));
            write_line(output, Eigen::Vector2i(faceBegIdx + 8, faceBegIdx + 1));

            write_line(output, Eigen::Vector2i(faceBegIdx + 3, faceBegIdx + 4));
            write_line(output, Eigen::Vector2i(faceBegIdx + 4, faceBegIdx + 5));
            write_line(output, Eigen::Vector2i(faceBegIdx + 5, faceBegIdx + 6));
            write_line(output, Eigen::Vector2i(faceBegIdx + 6, faceBegIdx + 3));

            write_line(output, Eigen::Vector2i(faceBegIdx + 3, faceBegIdx + 1));
            write_line(output, Eigen::Vector2i(faceBegIdx + 4, faceBegIdx + 2));
            write_line(output, Eigen::Vector2i(faceBegIdx + 5, faceBegIdx + 7));
            write_line(output, Eigen::Vector2i(faceBegIdx + 6, faceBegIdx + 8));
#endif

            //faceBegIdx += 8;
        }

//        inline void write_tri(const std::vector<Vector3> &triVerts, int &vertBegIdx, std::ofstream &output) {
//            if (triVerts.size() != 3) {
//                LOG::qpWarn("Invalid number of triangle vertex!");
//                return;
//            }
//            if (vertBegIdx <= 0) vertBegIdx = 1;
//            for (int i = 0; i < 3; ++i) {
//                output << "v " << triVerts[i].transpose() << "\n";
//            }
//            output << "f " << vertBegIdx << " " << vertBegIdx + 1 << " " << vertBegIdx + 2 << "\n";
//            vertBegIdx += 3;
//        }

        inline void
        write_tri(const Vector3 &triVert_0, const Vector3 &triVert_1, const Vector3 &triVert_2,
                  int &vertBegIdx, std::ofstream &output) {
            if (vertBegIdx <= 0) vertBegIdx = 1;

            output << "v " << triVert_0.transpose() << "\n";
            output << "v " << triVert_1.transpose() << "\n";
            output << "v " << triVert_2.transpose() << "\n";
            output << "f " << vertBegIdx << " " << vertBegIdx + 1 << " " << vertBegIdx + 2 << "\n";

            vertBegIdx += 3;
        }

        inline void writePointCloud(const std::vector<Vector3> &points, std::ofstream &output) {
            for (const auto &point: points)
                write_vertex(output, point);
        }

        inline void writePointCloud_xyz(const std::vector<Vector3> &points, std::ofstream &output) {
            for (const auto &point: points)
                write_vertex_to_xyz(output, point);
        }

        inline void
        writePointCloud(const std::vector<Vector3> &points, const std::vector<Vector3> &rgbs, std::ofstream &output) {
            for (size_t i = 0; i < points.size(); ++i)
                write_vertex(output, points[i], rgbs[i]);
        }

        inline void writePointCloud(const MatrixX &points, std::ofstream &output) {
            for (size_t i = 0; i < points.rows(); ++i)
                write_vertex(output, points.row(i));
        }

        inline void writePointCloud_xyz(const MatrixX &points, std::ofstream &output) {
            for (size_t i = 0; i < points.rows(); ++i)
                write_vertex_to_xyz(output, points.row(i));
        }

        inline void writePointCloud(const MatrixX &points, const std::vector<Vector3> &rgbs, std::ofstream &output) {
            for (size_t i = 0; i < points.rows(); ++i)
                write_vertex(output, points.row(i), rgbs[i]);
        }

        inline void writePointCloud(const Vector3 &point, const Vector3 &rgb, std::ofstream &output) {
            write_vertex(output, point, rgb);
        }
    } // namespace geometry::gvis

    namespace geometry {
        template<class T>
        struct Edge {
            using type = std::pair<T, T>;
        };

        // TODO: optimize template
        // An Axis Aligned Box (AAB) of a certain T -
        // to be initialized with a boxOrigin and boxEnd
        template<class T>
        struct AABox {
        public:
            T boxOrigin;
            T boxEnd;
            T boxWidth;

            //using T = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

            AABox() : boxOrigin(T()), boxEnd(T()), boxWidth(T()) {}

            AABox(const T &_boxOrigin, const T &_boxEnd) : boxOrigin(_boxOrigin),
                                                           boxEnd(_boxEnd),
                                                           boxWidth(_boxEnd - _boxOrigin) {}

            AABox(const MatrixX &points) {
                boxOrigin = points.colwise().minCoeff();
                boxEnd = points.colwise().maxCoeff();
                boxWidth = boxEnd - boxOrigin;
            }

            void scaleAndTranslate(const double &scale_factor = 1.0, const Vector3 &translation = Vector3::Zero()) {
                if (scale_factor == 1.0 && translation == Vector3::Zero()) return;

                // Calculate centroid point
                const T center = (boxOrigin + boxEnd) / 2.0;

                // Zoom and translation based on the centroid point
                const T scaled_min_point = (boxOrigin - center) * scale_factor + center + translation;
                const T scaled_max_point = (boxEnd - center) * scale_factor + center + translation;

                // Update coordinate
                update(scaled_min_point, scaled_max_point);
            }

            AABox(const AABox<T> &_box) {
                boxOrigin = _box.boxOrigin;
                boxEnd = _box.boxEnd;
                boxWidth = _box.boxWidth;
            }

            AABox &operator=(const AABox<T> &_box) {
                boxOrigin = _box.boxOrigin;
                boxEnd = _box.boxEnd;
                boxWidth = _box.boxWidth;

                return *this;
            }

            bool isInBox(const T &queryPoint) const {
                int dim = boxOrigin.rows();
                for (int i = 0; i < dim; ++i)
                    if (!(boxOrigin(i) <= queryPoint(i) && queryPoint(i) <= boxEnd(i))) return false;
                return true;
            }

            void output(const std::string &filename) const {
                std::ofstream out(filename);

                gvis::writeCube(boxOrigin, boxWidth, 0, out);

                out.close();
            }

        private:
            void update(const T &_boxOrigin, const T &_boxEnd) {
                boxOrigin = _boxOrigin;
                boxEnd = _boxEnd;
                boxWidth = boxEnd - boxOrigin;
            }
        };

        template<typename T>
        struct Triangle {
            T p1, p2, p3;
            T normal;
            double area;
            double dir;

            Triangle() {}

            Triangle(const T &_p1, const T &_p2, const T &_p3) : p1(_p1), p2(_p2),
                                                                 p3(_p3) {
                normal = (p2 - p1).cross(p3 - p1);
            }
        };

        struct Tet_ {
            typedef Vector3 Point;
            typedef std::array<Point, 4> Tetrahedron;
            typedef std::array<int, 4> TetNeighbor;
        };

        // 高斯采样
        template<typename T>
        struct GaussianSampleGen {
        private:
            unsigned int numSamples;

        public:
            explicit GaussianSampleGen(const unsigned int &_numSamples) : numSamples(_numSamples) {}

            std::vector<T> run(const AABox<T> &box) {
                static std::random_device rd;
                static std::mt19937 gen(rd());
                std::normal_distribution<double> dist(0.0, 1.0);

                const int dim = box.boxOrigin.rows();
                T mean = (box.boxEnd + box.boxOrigin) / 2.0;
                T stddev = (box.boxEnd - box.boxOrigin) / 6.0;

                std::vector<T> res;
                res.resize(numSamples);

                // std::ofstream out(".\\gaussian_points.xyz");

#pragma omp parallel for
                for (int i = 0; i < numSamples; ++i) {
                    // 生成高斯样本
                    T sample;
                    for (int j = 0; j < dim; ++j) {
                        sample(j) = mean(j) + stddev(j) * dist(gen);
                    }
                    res[i] = sample;
                    // 打印样本
                    // std::cout << "Sample " << i + 1 << ": (" << sample(0) << ", " << sample(1) << ", " << sample(2) << ")" << std::endl;
                    // out << sample(0) << " " << sample(1) << " " << sample(2) << std::endl;
                }
                return res;
            }
        };

        template<typename T>
        struct UniformSampleGen {
        private:
            unsigned int numSamples;

            constexpr static const int Base[3] = {
                    2, // X轴上的基数
                    3, // Y轴上的基数
                    5 // Z轴上的基数
            };

            // 生成Halton序列的第index个值
            double haltonSequence(int index, int base) const {
                double result = 0.0;
                double f = 1.0 / base;
                int i = index;

                while (i > 0) {
                    result += f * (i % base);
                    i = std::floor(i / base);
                    f /= base;
                }

                return result;
            }

            // 将Halton序列值映射到[min, max]范围内
            double mapToRange(double value, double min, double max) const {
                return min + value * (max - min);
            }

            // 在[minArea, maxArea]范围内进行蓝噪声采样
            std::vector<T> generateUniformSamples(const T &minArea,
                                                  const T &maxArea) const {
                std::vector<T> samples(numSamples);

                const int dim = minArea.rows(); // TODO: 使用Real内部的Rows, 实现constexpr
#pragma omp parallel for
                for (int i = 0; i < numSamples; ++i) {
                    T val;
                    for (int j = 0; j < dim; ++j) {
                        val(j) = mapToRange(haltonSequence(i, Base[j]), minArea(j), maxArea(j));
                    }
                    samples[i] = val;
//                    std::cout << "val = " << val.transpose() << std::endl;
                }

                return samples;
            }

        public:
            explicit UniformSampleGen(const unsigned int &_numSamples) : numSamples(_numSamples) {}

            // Generate uniform samples in the given 3D space using Sobol sequence
            std::vector<T>
            run(const AABox<T> &box) const {
                return generateUniformSamples(box.boxOrigin, box.boxEnd);
            }
        };

        template<typename T>
        struct UniformSampleGen_Sobol {
        private:
            unsigned int numSamples;

            // Sobol direction numbers for up to 21201 dimensions
            constexpr static const unsigned int sobolDirectionNumbers[][50] = {
                    // Direction numbers for 1 dimension
                    {1u, 0u},
                    // Direction numbers for 2 dimensions
                    {1u, 1u, 0u},
                    // Direction numbers for 3 dimensions
                    {1u, 3u, 1u, 0u},
                    // Direction numbers for 4 dimensions
                    {1u, 7u, 5u, 1u, 0u},
                    // ... direction numbers for higher dimensions
            };

            // Helper function to calculate the number of bits required to represent a value
            unsigned int countBits(unsigned int value) {
                unsigned int count = 0;
                while (value > 0) {
                    value >>= 1;
                    count++;
                }
                return count;
            }

            // Helper function to calculate the ith component of the Sobol sequence
            double sobolSample(unsigned int i, unsigned int n) {
                unsigned int bits = countBits(n);
                unsigned int directionCount =
                        sizeof(sobolDirectionNumbers) / sizeof(sobolDirectionNumbers[0]);

                if (i >= directionCount) {
                    std::cerr << "Sobol sequence not supported for " << i << " dimensions."
                              << std::endl;
                    return 0.0f;
                }

                const unsigned int *directionNumbers = sobolDirectionNumbers[i];
                unsigned int directionCountBits = countBits(directionNumbers[0]);

                if (bits > directionCountBits) {
                    std::cerr << "Sobol sequence not supported for " << bits << " bits."
                              << std::endl;
                    return 0.0f;
                }

                unsigned int result = 0;
                for (unsigned int j = 1; j <= bits; ++j) {
                    if ((n & (1u << (bits - j))) != 0) {
                        result ^= directionNumbers[j];
                    }
                }

                return static_cast<double>(result) / static_cast<double>(1u << bits);
            }

        public:
            explicit UniformSampleGen_Sobol(const unsigned int &_numSamples) : numSamples(_numSamples) {}

            // Generate uniform samples in the given 3D space using Sobol sequence
            std::vector<T>
            run(const AABox<T> &box) {
                const int dim = box.boxEnd.rows();
                unsigned int currentSample = 0;

                std::vector<T> samples;
                samples.resize(numSamples);

#pragma omp parallel for
                for (int i = 0; i < numSamples; ++i) {
                    T sample;
                    for (unsigned int k = 0; k < dim; ++k) {
                        sample(k) = box.boxOrigin(k) + sobolSample(k, i);
                        sample(k) *= box.boxEnd(k) - box.boxOrigin(k);
                    }
                    samples[i] = sample;
                }

                return samples;
            }
        };

    } // namespace geometry

NAMESPACE_END(GEOBOX)

#endif //GEOBOX_GEOMETRY_HPP
