add_lib(GEOBOX_DETAIL INTERFACE)

file(GLOB DETAIL_SOURCE "*.hpp")
target_sources(GEOBOX_DETAIL
        PRIVATE
        ${DETAIL_SOURCE}
)

target_include_directories(GEOBOX_DETAIL
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(GEOBOX_DETAIL
        INTERFACE
        Eigen3::Eigen
)