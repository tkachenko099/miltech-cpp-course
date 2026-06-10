add_library(ballistics src/ballistics.cpp)

target_include_directories(ballistics PUBLIC include)

add_executable(ballistics_cli src/main.cpp)

target_link_libraries(ballistics_cli PRIVATE ballistics)

add_executable(ballistics_tests tests/ballistics_tests.cpp)

target_link_libraries(ballistics_tests PRIVATE ballistics GTest::gtest_main)

include(GoogleTest)

gtest_discover_tests(ballistics_tests)
