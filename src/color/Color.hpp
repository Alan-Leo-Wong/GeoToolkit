//
// Created by Lei on 2/20/2024.
//

#ifndef GEOBOX_COLOR_HPP
#define GEOBOX_COLOR_HPP

#include <Config.hpp>
#include <BasicDataType.hpp>

NAMESPACE_BEGIN(GEOBOX)
    namespace color {
        using namespace detail;

        struct RGB {
            Vector3 rgb;

            RGB() = default;

            RGB(Vector3 _rgb) : rgb(std::move(_rgb)) {}

            RGB(float r, float g, float b) : rgb(Vector3(r, g, b)) {}

            Vector3 operator()() {
                return rgb;
            }

            Scalar operator()(int idx) {
                return rgb(idx);
            }
        };

        inline std::ostream &operator<<(std::ostream &out, const RGB &_rgb) {
            out << _rgb.rgb;
        }

        struct RGBA : public RGB {
            float alpha;

            RGBA() = default;

            RGBA(Vector3 _rgb, float _alpha) : RGB(_rgb), alpha(_alpha) {}

            RGBA(float r, float g, float b, float _alpha) : RGB(r, g, b), alpha(_alpha) {}
        };
    }
NAMESPACE_END(GEOBOX)

#endif //GEOBOX_COLOR_HPP
