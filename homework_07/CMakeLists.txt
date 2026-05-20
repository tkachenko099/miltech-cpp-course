cmake_minimum_required(VERSION 3.28)

project(homework_07 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(homework_07)

target_sources(homework_07
    PRIVATE
        src/main.cpp
    PUBLIC
        FILE_SET cxx_modules TYPE CXX_MODULES FILES
            modules/mission_types.ixx
            modules/interfaces.ixx
            modules/json_target_provider.ixx
            modules/file_config_loader.ixx
            modules/analytical_solver.ixx
            modules/factory.ixx
            modules/mission_processor.ixx
)

target_include_directories(homework_07 PRIVATE external)