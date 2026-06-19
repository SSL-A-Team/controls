find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(
  cpp_tests
  test_bangbang.cpp
  test_robot_model.cpp
  test_pivot_trajectory.cpp
)
target_link_libraries(
  cpp_tests
  ateam_controls
  GTest::gtest_main
)

gtest_discover_tests(cpp_tests)
