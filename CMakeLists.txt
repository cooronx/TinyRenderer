cmake_minimum_required(VERSION 3.24)
project(TinyRenderer)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

set(CMAKE_EXPORT_COMPILE_COMMANDS True)
include_directories(${PROJECT_SOURCE_DIR}/include)


add_executable(TinyRenderer main.cpp
        model.cpp
        tgaimage.cpp
        camera.cpp
        tinyrenderer.cpp
        include/tinyrenderer.h
)