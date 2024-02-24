//
// Created by Lei on 2/20/2024.
//

#ifndef GEOBOX_COLORMAP_HPP
#define GEOBOX_COLORMAP_HPP

#include <Config.hpp>
#include <Math.hpp>
#include <random>
#include "Color.hpp"
#include "ColorMapData.hpp"

NAMESPACE_BEGIN(GEOBOX)
    namespace color {

        class ColorMap {
        public:
            enum class INNER_CM {
                RAINBOW
            };

            ColorMap() = default;

            ColorMap(INNER_CM _cm) : inner_cm(_cm) {}

            RGB get_color(int index) {
                if (index < 0 || index >= 256)
                    clamp(index, 0, 256);
                switch (inner_cm) {
                    case INNER_CM::RAINBOW:
                        return RGB(ColorMapData::color_map_array[0][index * 3],
                                   ColorMapData::color_map_array[0][index * 3 + 1],
                                   ColorMapData::color_map_array[0][index * 3 + 2]
                        );
                }
            }

            static RGB get_color(INNER_CM cm, int index) {
                if (index < 0 || index >= 256)
                    clamp(index, 0, 256);
                switch (cm) {
                    case INNER_CM::RAINBOW:
                        return RGB(ColorMapData::color_map_array[0][index * 3],
                                   ColorMapData::color_map_array[0][index * 3 + 1],
                                   ColorMapData::color_map_array[0][index * 3 + 2]
                        );
                }
            }

            RGB get_color(int cm, int index) {
                if (index < 0 || index >= 256)
                    clamp(index, 0, 256);
                if (cm > new_cm.size()) return RGB(0, 0, 0);

                return RGB(new_cm[cm][index * 3],
                           new_cm[cm][index * 3 + 1],
                           new_cm[cm][index * 3 + 2]
                );
            }

            RGB get_random_color() {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> dis(0, 255);

                int random_index = dis(gen);

                switch (inner_cm) {
                    case INNER_CM::RAINBOW:
                        return RGB(ColorMapData::color_map_array[0][random_index * 3],
                                   ColorMapData::color_map_array[0][random_index * 3 + 1],
                                   ColorMapData::color_map_array[0][random_index * 3 + 2]
                        );
                }
            }

            static RGB get_random_color(INNER_CM cm) {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> dis(0, 255);

                int random_index = dis(gen);

                switch (cm) {
                    case INNER_CM::RAINBOW:
                        return RGB(ColorMapData::color_map_array[0][random_index * 3],
                                   ColorMapData::color_map_array[0][random_index * 3 + 1],
                                   ColorMapData::color_map_array[0][random_index * 3 + 2]
                        );
                }
            }

            RGB get_random_color(int cm) {
                if (cm > new_cm.size()) return RGB(0, 0, 0);

                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> dis(0, 255);

                int random_index = dis(gen);

                return RGB(new_cm[cm][random_index * 3],
                           new_cm[cm][random_index * 3 + 1],
                           new_cm[cm][random_index * 3 + 2]
                );
            }

            void creat_color_map(const char *filename) {
                std::vector<float> cm = ColorMapData::read_color_map_from_file(filename);
                new_cm.emplace_back(cm);
            }

        private:
            INNER_CM inner_cm = INNER_CM::RAINBOW;

            std::vector<std::vector<float>> new_cm;
        };

    }
NAMESPACE_END(GEOBOX)

#endif //GEOBOX_COLORMAP_HPP
