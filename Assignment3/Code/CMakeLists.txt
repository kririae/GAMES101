cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

find_package(OpenMP)

if (OPENMP_FOUND OR OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast")

include_directories(/usr/local/include ./include)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})

#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)