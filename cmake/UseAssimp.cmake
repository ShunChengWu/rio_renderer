# Use 4.1.0 to prevent this error: https://github.com/assimp/assimp/issues/2754
find_package(Assimp 4.1.0 QUIET)
OPTION(WITH_ASSIMP "Build with Assimp support?" ${Assimp_FOUND})

IF(WITH_ASSIMP)
    MESSAGE(STATUS "WITH Assimp")
    IF (NOT Assimp_FOUND)
      INCLUDE(ConfigureTimeDependency)
      add_configure_time_dependency(Assimp)
      find_package(Assimp 4.1.0 REQUIRED)
    ENDIF()
    
    INCLUDE_DIRECTORIES(${Assimp_INCLUDE_DIRS})
ENDIF()
