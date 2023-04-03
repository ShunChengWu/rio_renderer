
# Adds steps to download, configure and install dependencies at configure-time
function(add_configure_time_dependency name)
    if (NOT TARGET update-${name})
        MESSAGE("-build " ${name})
        configure_file(${CMAKE_SOURCE_DIR}/cmake/External_${name}.CMakeLists.txt.in ${CMAKE_BINARY_DIR}/external/${name}/download/CMakeLists.txt)
        execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external/${name}/download )
        if(result)
            message(FATAL_ERROR "CMake step for ${name} failed: ${result}")
        endif()

        MESSAGE("CMAKE_COMMAND: " ${CMAKE_COMMAND})
        execute_process(COMMAND ${CMAKE_COMMAND} --build .
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external/${name}/download )
        if(result)
            message(FATAL_ERROR "Build step for ${name} failed: ${result}")
        endif()

        execute_process(COMMAND make install
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external/${name}/src/${name}-build)
        execute_process(COMMAND make install
                RESULT_VARIABLE result
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external/${name}/download/${name}-build)
#        if(result)
#            message(FATAL_ERROR "Build step for ${name} failed: ${result}")
#        endif()

        # Add update target to get new sources and recompile
        add_custom_target(update-${name}
                COMMAND ${CMAKE_COMMAND} --build . --target ${name}-update
                COMMAND ${CMAKE_COMMAND} --build . --target all
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/external/${name}/download
                )
    ELSE()
        MESSAGE("-skip " ${name})
    ENDIF()
endfunction()