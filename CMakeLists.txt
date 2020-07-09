cmake_minimum_required(VERSION 2.8.3)
project(SFM)


# CXX flags 
set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -w -std=c++11 -fpermissive") # -fPIC -fpermissive

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ./src
  "/usr/include/eigen3"
)
####### OpenCV #######
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

####### Ceres #######
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_subdirectory(src bin)

