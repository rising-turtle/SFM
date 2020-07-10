project(SFM)

###########
## Ceres ##
###########

add_executable(sfm sfm.cpp main_sfm.cpp mean_std.cpp)
target_link_libraries(sfm ${OpenCV_LIBS} ${CERES_LIBRARIES})
