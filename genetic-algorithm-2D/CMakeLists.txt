cmake_minimum_required(VERSION 3.12)
project(genetic-algoritm-2D VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(atta 0.0.0.3 REQUIRED)
include_directories(${atta_INCLUDE_DIRS})

add_library(projectScript SHARED
    src/projectScript.cpp
)

add_library(robotScript SHARED
    src/robotScript.cpp
)

add_library(geneComponent SHARED
    src/geneComponent.cpp
)
