add_lib(GEOBOX_VORONOI INTERFACE)
target_sources(GEOBOX_VORONOI
        PRIVATE
        Voronoi.hpp
)

add_lib(GEOBOX_CGAL_VORONOI)
target_sources(GEOBOX_CGAL_VORONOI
        PRIVATE
        CGALVoronoi.hpp
        CGALVoronoi.cpp
)
target_link_libraries(GEOBOX_CGAL_VORONOI
    INTERFACE
    GEOBOX::VORONOI
)
target_link_libraries(GEOBOX_CGAL_VORONOI
    PRIVATE
    CGAL::CGAL
    CGAL::CGAL_Core
)

add_lib(GEOBOX_GEO_VORONOI)
target_sources(GEOBOX_GEO_VORONOI
        PRIVATE
        GEOVoronoi.hpp
        GEOVoronoi.cpp
)
target_link_libraries(GEOBOX_GEO_VORONOI
        INTERFACE
        GEOBOX::VORONOI
)
target_link_libraries(GEOBOX_GEO_VORONOI
        PRIVATE
        geogram::geogram
)