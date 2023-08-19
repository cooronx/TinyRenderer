cmake_minimum_required(VERSION 3.24)
project(TinyRenderer)

set(CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/include)


add_executable(TinyRenderer main.cpp
        model.cpp
        tgaimage.cpp)
