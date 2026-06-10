cmake_minimum_required(VERSION 3.22)

project(homework_09 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(homework_09
    src/main.cpp
    src/MissionProcessor.cpp

    src/solvers/AnalyticalSolver.cpp
    src/solvers/BallisticTable.cpp
    src/solvers/TableSolver.cpp

    src/providers/JsonTargetProvider.cpp

    src/config/FileConfigLoader.cpp
    src/config/ComponentFactory.cpp

    src/states/DroneStates.cpp
)

target_include_directories(homework_09
    PRIVATE
        include
        external
)