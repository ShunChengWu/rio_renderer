# Use 4.1.0 to prevent this error: https://github.com/assimp/assimp/issues/2754
find_package(Assimp 4.1.0 QUIET
    PATHS
    "${PROJECT_SOURCE_DIR}/external/lib/cmake/assimp-4.1"
)
OPTION(WITH_ASSIMP "Build with Assimp support?" ${Assimp_FOUND})
MESSAGE("WITH_ASSIMP: " ${WITH_ASSIMP})
MESSAGE("Assimp_INCLUDE_DIRS1: " ${ASSIMP_INCLUDE_DIRS})
IF(WITH_ASSIMP)
    MESSAGE(STATUS "WITH Assimp")
    IF (NOT Assimp_FOUND)
      INCLUDE(ConfigureTimeDependency)
      add_configure_time_dependency(Assimp)
      find_package(Assimp 4.1.0 REQUIRED
        PATHS
        "${PROJECT_SOURCE_DIR}/external/lib/cmake/assimp-4.1"
        )
    ENDIF()
    
    INCLUDE_DIRECTORIES(${ASSIMP_INCLUDE_DIRS})
    MESSAGE("Assimp_INCLUDE_DIRS1: " ${ASSIMP_INCLUDE_DIRS})
ENDIF()
