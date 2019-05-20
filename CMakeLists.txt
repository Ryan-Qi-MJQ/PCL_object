cmake_minimum_required(VERSION 3.13)
project(dianyun_danwei)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(dianyun_danwei main.cpp)
target_link_libraries(dianyun_danwei ${PCL_LIBRARIES})

add_executable(chong_jian chong_jian.cpp )
target_link_libraries(chong_jian ${PCL_LIBRARIES})