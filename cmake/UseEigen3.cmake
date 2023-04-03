###################
# UseEigen3.cmake #
###################
#set(CMAKE_FIND_DEBUG_MODE TRUE)
find_package(Eigen3 3 QUIET
        PATHS
        "/usr/local/share/eigen3"
        "/usr/local/share/eigen3/cmake"
        NO_DEFAULT_PATH)
#set(CMAKE_FIND_DEBUG_MODE FALSE)
MESSAGE("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
OPTION(WITH_EIGEN "Build with libEIGEN support?" OFF)
IF(WITH_EIGEN)
  IF(NOT Eigen3_FOUND)
    INCLUDE(ConfigureTimeDependency)
    add_configure_time_dependency(Eigen3)
    FILE(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/external/Eigen3)
    SET(Eigen3_DIR ${PROJECT_SOURCE_DIR}/external/share/eigen3/cmake)
    find_package(Eigen3 3 REQUIRED)
  ENDIF()
  IF(NOT EIGEN3_INCLUDE_DIR)
    IF(TARGET Eigen3::Eigen)
      MESSAGE(STATUS "found Eigen3::Eigen")
      GET_TARGET_PROPERTY(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
      ELIF(EIGEN_INCLUDE_DIRS)
      SET(EIGEN3_INCLUDE_DIR ${EIGEN_INCLUDE_DIRS})
    ELSE()
      MESSAGE(WARNING "cannot find EIGEN3_INCLUDE_DIR")
    ENDIF()
  ENDIF()
  include_directories(${EIGEN3_INCLUDE_DIR})
  MESSAGE("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
  ADD_DEFINITIONS(-DCOMPILE_WITH_EIGEN)
ENDIF()


