SET(targetname rio_renderer)

ADD_EXECUTABLE(${targetname} main.cpp renderer.cpp)
TARGET_LINK_LIBRARIES(${targetname}
        PUBLIC libDataLoader
        PUBLIC GUI3D)
INCLUDE(LinkEigen3)
INCLUDE(LinkOpenCV)
INCLUDE(LinkAssimp)
INCLUDE(LinkJson)