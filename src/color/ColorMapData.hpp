//
// Created by Lei on 2/20/2024.
//

#ifndef GEOBOX_COLORMAPDATA_HPP
#define GEOBOX_COLORMAPDATA_HPP

#include <Config.hpp>
//#include <array>
#include <vector>

NAMESPACE_BEGIN(GEOBOX)
    namespace color {

        class ColorMapData {
        private:
            constexpr static float color_map_array[1][256 * 3] = {
                    0.500000, 0.000000, 1.000000,
                    0.492157, 0.012320, 0.999981,
                    0.484314, 0.024637, 0.999924,
                    0.476471, 0.036951, 0.999829,
                    0.468627, 0.049260, 0.999696,
                    0.460784, 0.061561, 0.999526,
                    0.452941, 0.073853, 0.999317,
                    0.445098, 0.086133, 0.999070,
                    0.437255, 0.098400, 0.998786,
                    0.429412, 0.110653, 0.998464,
                    0.421569, 0.122888, 0.998103,
                    0.413725, 0.135105, 0.997705,
                    0.405882, 0.147302, 0.997269,
                    0.398039, 0.159476, 0.996795,
                    0.390196, 0.171626, 0.996284,
                    0.382353, 0.183750, 0.995734,
                    0.374510, 0.195845, 0.995147,
                    0.366667, 0.207912, 0.994522,
                    0.358824, 0.219946, 0.993859,
                    0.350980, 0.231948, 0.993159,
                    0.343137, 0.243914, 0.992421,
                    0.335294, 0.255843, 0.991645,
                    0.327451, 0.267733, 0.990831,
                    0.319608, 0.279583, 0.989980,
                    0.311765, 0.291390, 0.989092,
                    0.303922, 0.303153, 0.988165,
                    0.296078, 0.314870, 0.987202,
                    0.288235, 0.326539, 0.986201,
                    0.280392, 0.338158, 0.985162,
                    0.272549, 0.349727, 0.984086,
                    0.264706, 0.361242, 0.982973,
                    0.256863, 0.372702, 0.981823,
                    0.249020, 0.384106, 0.980635,
                    0.241176, 0.395451, 0.979410,
                    0.233333, 0.406737, 0.978148,
                    0.225490, 0.417960, 0.976848,
                    0.217647, 0.429121, 0.975512,
                    0.209804, 0.440216, 0.974139,
                    0.201961, 0.451244, 0.972728,
                    0.194118, 0.462204, 0.971281,
                    0.186275, 0.473094, 0.969797,
                    0.178431, 0.483911, 0.968276,
                    0.170588, 0.494656, 0.966718,
                    0.162745, 0.505325, 0.965124,
                    0.154902, 0.515918, 0.963493,
                    0.147059, 0.526432, 0.961826,
                    0.139216, 0.536867, 0.960122,
                    0.131373, 0.547220, 0.958381,
                    0.123529, 0.557489, 0.956604,
                    0.115686, 0.567675, 0.954791,
                    0.107843, 0.577774, 0.952942,
                    0.100000, 0.587785, 0.951057,
                    0.092157, 0.597707, 0.949135,
                    0.084314, 0.607539, 0.947177,
                    0.076471, 0.617278, 0.945184,
                    0.068627, 0.626924, 0.943154,
                    0.060784, 0.636474, 0.941089,
                    0.052941, 0.645928, 0.938988,
                    0.045098, 0.655284, 0.936852,
                    0.037255, 0.664540, 0.934680,
                    0.029412, 0.673696, 0.932472,
                    0.021569, 0.682749, 0.930229,
                    0.013725, 0.691698, 0.927951,
                    0.005882, 0.700543, 0.925638,
                    0.001961, 0.709281, 0.923289,
                    0.009804, 0.717912, 0.920906,
                    0.017647, 0.726434, 0.918487,
                    0.025490, 0.734845, 0.916034,
                    0.033333, 0.743145, 0.913545,
                    0.041176, 0.751332, 0.911023,
                    0.049020, 0.759405, 0.908465,
                    0.056863, 0.767363, 0.905873,
                    0.064706, 0.775204, 0.903247,
                    0.072549, 0.782928, 0.900587,
                    0.080392, 0.790532, 0.897892,
                    0.088235, 0.798017, 0.895163,
                    0.096078, 0.805381, 0.892401,
                    0.103922, 0.812622, 0.889604,
                    0.111765, 0.819740, 0.886774,
                    0.119608, 0.826734, 0.883910,
                    0.127451, 0.833602, 0.881012,
                    0.135294, 0.840344, 0.878081,
                    0.143137, 0.846958, 0.875117,
                    0.150980, 0.853444, 0.872120,
                    0.158824, 0.859800, 0.869089,
                    0.166667, 0.866025, 0.866025,
                    0.174510, 0.872120, 0.862929,
                    0.182353, 0.878081, 0.859800,
                    0.190196, 0.883910, 0.856638,
                    0.198039, 0.889604, 0.853444,
                    0.205882, 0.895163, 0.850217,
                    0.213725, 0.900587, 0.846958,
                    0.221569, 0.905873, 0.843667,
                    0.229412, 0.911023, 0.840344,
                    0.237255, 0.916034, 0.836989,
                    0.245098, 0.920906, 0.833602,
                    0.252941, 0.925638, 0.830184,
                    0.260784, 0.930229, 0.826734,
                    0.268627, 0.934680, 0.823253,
                    0.276471, 0.938988, 0.819740,
                    0.284314, 0.943154, 0.816197,
                    0.292157, 0.947177, 0.812622,
                    0.300000, 0.951057, 0.809017,
                    0.307843, 0.954791, 0.805381,
                    0.315686, 0.958381, 0.801714,
                    0.323529, 0.961826, 0.798017,
                    0.331373, 0.965124, 0.794290,
                    0.339216, 0.968276, 0.790532,
                    0.347059, 0.971281, 0.786745,
                    0.354902, 0.974139, 0.782928,
                    0.362745, 0.976848, 0.779081,
                    0.370588, 0.979410, 0.775204,
                    0.378431, 0.981823, 0.771298,
                    0.386275, 0.984086, 0.767363,
                    0.394118, 0.986201, 0.763398,
                    0.401961, 0.988165, 0.759405,
                    0.409804, 0.989980, 0.755383,
                    0.417647, 0.991645, 0.751332,
                    0.425490, 0.993159, 0.747253,
                    0.433333, 0.994522, 0.743145,
                    0.441176, 0.995734, 0.739009,
                    0.449020, 0.996795, 0.734845,
                    0.456863, 0.997705, 0.730653,
                    0.464706, 0.998464, 0.726434,
                    0.472549, 0.999070, 0.722186,
                    0.480392, 0.999526, 0.717912,
                    0.488235, 0.999829, 0.713610,
                    0.496078, 0.999981, 0.709281,
                    0.503922, 0.999981, 0.704926,
                    0.511765, 0.999829, 0.700543,
                    0.519608, 0.999526, 0.696134,
                    0.527451, 0.999070, 0.691698,
                    0.535294, 0.998464, 0.687237,
                    0.543137, 0.997705, 0.682749,
                    0.550980, 0.996795, 0.678235,
                    0.558824, 0.995734, 0.673696,
                    0.566667, 0.994522, 0.669131,
                    0.574510, 0.993159, 0.664540,
                    0.582353, 0.991645, 0.659925,
                    0.590196, 0.989980, 0.655284,
                    0.598039, 0.988165, 0.650618,
                    0.605882, 0.986201, 0.645928,
                    0.613725, 0.984086, 0.641213,
                    0.621569, 0.981823, 0.636474,
                    0.629412, 0.979410, 0.631711,
                    0.637255, 0.976848, 0.626924,
                    0.645098, 0.974139, 0.622113,
                    0.652941, 0.971281, 0.617278,
                    0.660784, 0.968276, 0.612420,
                    0.668627, 0.965124, 0.607539,
                    0.676471, 0.961826, 0.602635,
                    0.684314, 0.958381, 0.597707,
                    0.692157, 0.954791, 0.592758,
                    0.700000, 0.951057, 0.587785,
                    0.707843, 0.947177, 0.582791,
                    0.715686, 0.943154, 0.577774,
                    0.723529, 0.938988, 0.572735,
                    0.731373, 0.934680, 0.567675,
                    0.739216, 0.930229, 0.562593,
                    0.747059, 0.925638, 0.557489,
                    0.754902, 0.920906, 0.552365,
                    0.762745, 0.916034, 0.547220,
                    0.770588, 0.911023, 0.542053,
                    0.778431, 0.905873, 0.536867,
                    0.786275, 0.900587, 0.531659,
                    0.794118, 0.895163, 0.526432,
                    0.801961, 0.889604, 0.521185,
                    0.809804, 0.883910, 0.515918,
                    0.817647, 0.878081, 0.510631,
                    0.825490, 0.872120, 0.505325,
                    0.833333, 0.866025, 0.500000,
                    0.841176, 0.859800, 0.494656,
                    0.849020, 0.853444, 0.489293,
                    0.856863, 0.846958, 0.483911,
                    0.864706, 0.840344, 0.478512,
                    0.872549, 0.833602, 0.473094,
                    0.880392, 0.826734, 0.467658,
                    0.888235, 0.819740, 0.462204,
                    0.896078, 0.812622, 0.456733,
                    0.903922, 0.805381, 0.451244,
                    0.911765, 0.798017, 0.445738,
                    0.919608, 0.790532, 0.440216,
                    0.927451, 0.782928, 0.434676,
                    0.935294, 0.775204, 0.429121,
                    0.943137, 0.767363, 0.423549,
                    0.950980, 0.759405, 0.417960,
                    0.958824, 0.751332, 0.412356,
                    0.966667, 0.743145, 0.406737,
                    0.974510, 0.734845, 0.401102,
                    0.982353, 0.726434, 0.395451,
                    0.990196, 0.717912, 0.389786,
                    0.998039, 0.709281, 0.384106,
                    1.000000, 0.700543, 0.378411,
                    1.000000, 0.691698, 0.372702,
                    1.000000, 0.682749, 0.366979,
                    1.000000, 0.673696, 0.361242,
                    1.000000, 0.664540, 0.355491,
                    1.000000, 0.655284, 0.349727,
                    1.000000, 0.645928, 0.343949,
                    1.000000, 0.636474, 0.338158,
                    1.000000, 0.626924, 0.332355,
                    1.000000, 0.617278, 0.326539,
                    1.000000, 0.607539, 0.320710,
                    1.000000, 0.597707, 0.314870,
                    1.000000, 0.587785, 0.309017,
                    1.000000, 0.577774, 0.303153,
                    1.000000, 0.567675, 0.297277,
                    1.000000, 0.557489, 0.291390,
                    1.000000, 0.547220, 0.285492,
                    1.000000, 0.536867, 0.279583,
                    1.000000, 0.526432, 0.273663,
                    1.000000, 0.515918, 0.267733,
                    1.000000, 0.505325, 0.261793,
                    1.000000, 0.494656, 0.255843,
                    1.000000, 0.483911, 0.249883,
                    1.000000, 0.473094, 0.243914,
                    1.000000, 0.462204, 0.237935,
                    1.000000, 0.451244, 0.231948,
                    1.000000, 0.440216, 0.225951,
                    1.000000, 0.429121, 0.219946,
                    1.000000, 0.417960, 0.213933,
                    1.000000, 0.406737, 0.207912,
                    1.000000, 0.395451, 0.201882,
                    1.000000, 0.384106, 0.195845,
                    1.000000, 0.372702, 0.189801,
                    1.000000, 0.361242, 0.183750,
                    1.000000, 0.349727, 0.177691,
                    1.000000, 0.338158, 0.171626,
                    1.000000, 0.326539, 0.165554,
                    1.000000, 0.314870, 0.159476,
                    1.000000, 0.303153, 0.153392,
                    1.000000, 0.291390, 0.147302,
                    1.000000, 0.279583, 0.141206,
                    1.000000, 0.267733, 0.135105,
                    1.000000, 0.255843, 0.128999,
                    1.000000, 0.243914, 0.122888,
                    1.000000, 0.231948, 0.116773,
                    1.000000, 0.219946, 0.110653,
                    1.000000, 0.207912, 0.104528,
                    1.000000, 0.195845, 0.098400,
                    1.000000, 0.183750, 0.092268,
                    1.000000, 0.171626, 0.086133,
                    1.000000, 0.159476, 0.079994,
                    1.000000, 0.147302, 0.073853,
                    1.000000, 0.135105, 0.067708,
                    1.000000, 0.122888, 0.061561,
                    1.000000, 0.110653, 0.055411,
                    1.000000, 0.098400, 0.049260,
                    1.000000, 0.086133, 0.043107,
                    1.000000, 0.073853, 0.036951,
                    1.000000, 0.061561, 0.030795,
                    1.000000, 0.049260, 0.024637,
                    1.000000, 0.036951, 0.018479,
                    1.000000, 0.024637, 0.012320,
                    1.000000, 0.012320, 0.006160,
                    1.000000, 0.000000, 0.000000
            };

            // TODO
            static std::vector<float> read_color_map_from_file(const char *filename) {

            }

            friend class ColorMap;
        };

    }
NAMESPACE_END(GEOBOX)

#endif //GEOBOX_COLORMAPDATA_HPP
