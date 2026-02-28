find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(
  cpp_tests
  test_add.cpp
  test_bangbang.cpp
  test_robot_model.cpp
)
target_link_libraries(
  cpp_tests
  ateam_controls
  GTest::gtest_main
)

gtest_discover_tests(cpp_tests)
