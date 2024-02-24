#pragma once

#include <Config.hpp>
#include <numeric>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unordered_set>

namespace std {
    template<typename T, size_t N>
    struct hash<array<T, N> > {
        typedef array<T, N> argument_type;
        typedef size_t result_type;

        result_type operator()(const argument_type &a) const {
            hash<T> hasher;
            result_type h = 0;
            for (result_type i = 0; i < N; ++i) {
                h = h * 31 + hasher(a[i]);
            }
            return h;
        }
    };
}

NAMESPACE_BEGIN(GEOBOX)
    namespace detail {
        typedef unsigned int uint;
        typedef size_t index_t;

#ifdef USING_FLOAT
        typedef float Scalar;
        constexpr Scalar EQUAL_EPSILON = 1e-6;
        constexpr Scalar ZERO_EPSILON = 1e-6;
        const Scalar DINF = std::numeric_limits<float>::max();
#else
        typedef double Scalar;
        constexpr Scalar EQUAL_EPSILON = 1e-9;
        constexpr Scalar ZERO_EPSILON = 1e-9;
        const Scalar DINF = std::numeric_limits<double>::max();
        const Scalar DMIN = std::numeric_limits<double>::min(); // 正数最小值
        const Scalar DNMIN = std::numeric_limits<double>::lowest();
#endif

        template<typename _Scalar, int Dim>
        using Vector = typename Eigen::Matrix<_Scalar, Dim, 1>;

        using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

        using VectorXi = Eigen::VectorXi;

        using Vector2 = Eigen::Matrix<Scalar, 2, 1>;

        using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

        using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

        using Vector3i = Eigen::Vector3i;

        using Array3 = Eigen::Array<Scalar, 3, 1>;

        template<typename _Scalar, int Rows, int Cols>
        using Matrix = typename Eigen::Matrix<_Scalar, Rows, Cols>;

        using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

        using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

        using MatrixXi = Eigen::MatrixXi;

        template<typename _Scalar, int Num>
        using ArrayX = typename Eigen::Array<_Scalar, Num, 1>;

        template<typename _Scalar, int Rows, int Cols>
        using ArrayXX = typename Eigen::Array<_Scalar, Rows, Cols>;

        template<typename T>
        using _SpMatrix = Eigen::SparseMatrix<T>;
        using SpMatrix = _SpMatrix<Scalar>;
        using SpMatrixi = _SpMatrix<int>;

        template<typename T>
        using _SpTriplet = Eigen::Triplet<T>;
        using SpTriplet = _SpTriplet<Scalar>;
        using SpITriplet = _SpTriplet<int>;
    }
NAMESPACE_END(GEOBOX)

template<>
struct std::less<GEOBOX::detail::ArrayX<GEOBOX::detail::Scalar, 4>> {
public:
    using Array4 = GEOBOX::detail::ArrayX<GEOBOX::detail::Scalar, 4>;

    bool operator()(const Array4 &a, const Array4 &b) const {
        for (int i = 0; i < a.size(); ++i) {
            if (fabs(a[i] - b[i]) < 1e-9) continue;

            if (a[i] < b[i]) return true;
            else if (a[i] > b[i]) return false;
        }
        return false;
    }
};

template<>
struct std::less<GEOBOX::detail::VectorX> {
public:
    using VectorX = GEOBOX::detail::VectorX;

    bool operator()(const VectorX &a, const VectorX &b) const {
        for (int i = 0; i < a.size(); ++i) {
            if (fabs(a[i] - b[i]) < 1e-9) continue;

            if (a[i] < b[i]) return true;
            else if (a[i] > b[i]) return false;
        }
        return false;
    }
};

template<>
struct std::less<GEOBOX::detail::Vector3> {
public:
    using Vector3 = GEOBOX::detail::Vector3;

    bool operator()(const Vector3 &a, const Vector3 &b) const {
        for (int i = 0; i < 3; ++i) {
            if (fabs(a[i] - b[i]) < 1e-6) continue;

            if (a[i] < b[i]) return true;
            else if (a[i] > b[i]) return false;
        }
        return false;
    }
};

template<>
struct std::less<GEOBOX::detail::Vector4> {
public:
    using Vector4 = GEOBOX::detail::Vector4;

    bool operator()(const Vector4 &a, const Vector4 &b) const {
        for (int i = 0; i < 4; ++i) {
            if (fabs(a[i] - b[i]) < 1e-9) continue;

            if (a[i] < b[i]) return true;
            else if (a[i] > b[i]) return false;
        }
        return false;
    }
};
