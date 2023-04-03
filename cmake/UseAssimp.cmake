
find_package(Assimp QUIET)
OPTION(WITH_ASSIMP "Build with Assimp support?" ${Assimp_FOUND})

IF(WITH_ASSIMP)
    MESSAGE(STATUS "WITH Assimp")
    find_package(Assimp 4.1.0 REQUIRED) # To prevent this error https://github.com/assimp/assimp/issues/2754
    INCLUDE_DIRECTORIES(${Assimp_INCLUDE_DIRS})
ENDIF()