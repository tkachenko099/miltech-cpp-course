cmake_minimum_required(VERSION 3.28)

project(homework_08 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(homework_08
    src/main.cpp
    src/MissionProcessor.cpp
    src/solvers/AnalyticalSolver.cpp
    src/providers/JsonTargetProvider.cpp
    src/config/FileConfigLoader.cpp
    src/config/ComponentFactory.cpp
)

target_include_directories(homework_08 PRIVATE
    include
    external
)