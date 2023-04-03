cmake_minimum_required(VERSION 2.8)
project(RIO_Renderer C CXX)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
# =============================== FLAGS ===============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DCOMPILEDWITHC11)
#set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -O1 -march=native")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O1 -march=native")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -march=native")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3 -march=native")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


# =============================== module paths ===============================
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
# =============================== packages ===============================
SET(WITH_ASSIMP TRUE)
SET(WITH_OPENCV TRUE)
SET(WITH_EIGEN TRUE)
SET(WITH_JSON TRUE)
INCLUDE(UseEigen3)
INCLUDE(UseOpenCV)
INCLUDE(UseAssimp)
INCLUDE(UseJson)

# =============================== include directories ===============================
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

ADD_SUBDIRECTORY(libGUI3D)
ADD_SUBDIRECTORY(libDataLoader)
# =============================== build examples ===============================
ADD_SUBDIRECTORY(app)