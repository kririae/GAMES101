cmake_minimum_required(VERSION 3.10)
project(RayTracing)

find_package(OpenMP)

if (OPENMP_FOUND OR OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

set(CMAKE_CXX_STANDARD 17)
#add_compile_options(-O3)
#add_compile_options(-ftree-vectorize -msse2)
#add_compile_options(-ftree-vectorizer-verbose=1)
add_compile_options(-mtune=znver2)
add_compile_options(-march=znver2)

add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)
