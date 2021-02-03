cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND OR OpenMP_CXX_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

include_directories(/usr/local/include)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp ../Test.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
