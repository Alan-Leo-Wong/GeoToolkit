if (NOT TARGET Eigen3::Eigen)
    include(eigen)
endif ()

if (NOT TARGET igl::core)
    include(libigl)
endif ()

if(TARGET gmp::gmp)
    include(gmp)
endif()

if(TARGET mpfr::mpfr)
    include(mpfr)
endif()

if(NOT TARGET geogram::geogram)
    include(geogram)
endif()

if (NOT TARGET Boost::boost)
    include(boost)
endif ()

if (NOT TARGET CGAL::CGAL)
    include(cgal)
endif ()

if (NOT TARGET CLI11::CLI11)
    include(CLI)
    target_compile_definitions(CLI11 INTERFACE -DCLI11_STD_OPTIONAL=0)
    target_compile_definitions(CLI11 INTERFACE -DCLI11_EXPERIMENTAL_OPTIONAL=0)
endif ()

if (NOT TARGET spdlog)
    include(spdlog)
endif ()