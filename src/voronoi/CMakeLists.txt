add_lib(GEOToolKit_VORONOI INTERFACE)
target_sources(GEOToolKit_VORONOI
        PRIVATE
        Voronoi.hpp
)

add_lib(GEOToolKit_CGAL_VORONOI)
target_sources(GEOToolKit_CGAL_VORONOI
        PRIVATE
        CGALVoronoi.hpp
        CGALVoronoi.cpp
)
target_link_libraries(GEOToolKit_CGAL_VORONOI
    INTERFACE
    GEOToolKit::VORONOI
)
target_link_libraries(GEOToolKit_CGAL_VORONOI
    PRIVATE
    CGAL::CGAL
    CGAL::CGAL_Core
)

add_lib(GEOToolKit_GEO_VORONOI)
target_sources(GEOToolKit_GEO_VORONOI
        PRIVATE
        GEOVoronoi.hpp
        GEOVoronoi.cpp
)
target_link_libraries(GEOToolKit_GEO_VORONOI
        INTERFACE
        GEOToolKit::VORONOI
)
target_link_libraries(GEOToolKit_GEO_VORONOI
        PRIVATE
        geogram::geogram
)