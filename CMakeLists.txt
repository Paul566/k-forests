cmake_minimum_required(VERSION 3.27)
project(k_forests)

set(CMAKE_CXX_STANDARD 17)

add_executable(k_forests main.cpp
        GraphicMatroid.h
        GraphicMatroid.cpp
        Tester.h
        Tester.cpp)
