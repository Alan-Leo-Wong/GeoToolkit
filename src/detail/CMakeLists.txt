add_lib(GEOToolKit_DETAIL INTERFACE)

file(GLOB DETAIL_SOURCE "*.hpp")
target_sources(GEOToolKit_DETAIL
        PRIVATE
        ${DETAIL_SOURCE}
)

target_include_directories(GEOToolKit_DETAIL
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(GEOToolKit_DETAIL
        INTERFACE
        Eigen3::Eigen
)