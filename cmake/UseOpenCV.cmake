###################
# UseOpenCV.cmake #
###################
find_package(OpenCV QUIET)
OPTION(WITH_OpenCV "Build with OpenCV support?" ${OpenCV_FOUND})
MESSAGE("OpenCV_FOUND: " ${OpenCV_FOUND})
IF(WITH_OpenCV)
  IF(NOT OpenCV_FOUND)
      INCLUDE(ConfigureTimeDependency)
      add_configure_time_dependency(OpenCV)
      find_package(OpenCV REQUIRED)
  ENDIF()
  INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIB_DIRS})
  ADD_DEFINITIONS(-DCOMPILE_WITH_OPENCV)
ELSE()
  ADD_DEFINITIONS(-DCOMPILE_WITHOUT_OPENCV)
ENDIF()
