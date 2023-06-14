cmake_minimum_required(VERSION 3.21)
project(SPA_graphs)

set(CMAKE_CXX_STANDARD 20)

add_library(STATIC
        algorithms.cpp)

add_executable(SPA_graphs main.cpp graph.cpp static/enum.cpp)
