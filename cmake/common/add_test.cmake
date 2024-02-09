function(add_test module_name)
    #    if (NOT TARGET ${module_name})
    #        message(FATAL_ERROR "'${module_name}' is not a CMake target")
    #        return()
    #    endif ()

    add_executable(test_${module_name}
            ${ARGN}
    )

    # For IDEs that present targets using a folder hierarchy,
    # this property specifies the name of the folder to place the target under
    set_target_properties(test_${module_name} PROPERTIES FOLDER Geobox_UnitTests)

    # Output directory
    set_target_properties(test_${module_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Tests")

    # include(CTest)
    # TODO: Integrate with Cache2

endfunction()