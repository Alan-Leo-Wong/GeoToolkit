add_lib(GEOBOX_VIEWER)

target_sources(GEOBOX_VIEWER
        PRIVATE
        MeshViewer.hpp
        MeshViewer.cpp
)

#target_include_directories(GEOBOX_VIEWER
#        PRIVATE
#        ${CMAKE_SOURCE_DIR}/src
#)

target_link_libraries(GEOBOX_VIEWER
        PUBLIC
#        GEOBOX::DETAIL
        igl::core
        igl::glfw
        igl::imgui
)

if (ENABLE_TEST)
    # Find all .cpp files in the 'test' subdirectory
    file(GLOB test_sources "test/*.cpp")

    # Use foreach to process each test source file
    foreach (test_source ${test_sources})
        get_filename_component(test_source_name ${test_source} NAME_WE) # 不需要后缀

        # Generate library name for the test source file
        set(test_module_name "${test_source_name}")

        # Check if module_name ends with "Test" and remove it
        string(REGEX REPLACE "Test$" "" modified_test_module_name ${test_module_name})
        if (modified_test_module_name)
            set(test_module_name "${modified_test_module_name}")
        endif ()

        # Add the test executable using my_add_test function
        add_test(${test_module_name} ${test_source})
    endforeach ()

    target_link_libraries(test_MeshViewer PRIVATE GEOBOX_VIEWER CLI11)

endif ()