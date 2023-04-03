####################
# LinkEigen3.cmake #
####################
if(TARGET Eigen3::Eigen)
  target_link_libraries(${targetname} PUBLIC Eigen3::Eigen)
  TARGET_COMPILE_DEFINITIONS(${targetname} PUBLIC COMPILE_WITH_EIGEN)
elseif(EIGEN3_INCLUDE_DIR)
  target_include_directories(${targetname} PUBLIC ${EIGEN3_INCLUDE_DIR})
  TARGET_COMPILE_DEFINITIONS(${targetname} PUBLIC COMPILE_WITH_EIGEN)
else()
  MESSAGE(FATAL_ERROR "cannot find eigen3")
endif()