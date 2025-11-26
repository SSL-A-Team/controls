cmake_minimum_required(VERSION 3.28)

project(ateam_controls LANGUAGES CXX)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)

set(RUST_TARGET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/target)

if((CMAKE_BUILD_TYPE STREQUAL "Release") OR (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo"))
  set(IMPORTED_LIB_PATH ${RUST_TARGET_DIR}/release/libateam_controls_c.a)
else()
  set(IMPORTED_LIB_PATH ${RUST_TARGET_DIR}/debug/libateam_controls_c.a)
endif()

# Glob all Rust source files and Cargo.toml files to track as dependencies
file(GLOB_RECURSE RUST_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/ateam_controls/src/*.rs"
  "${CMAKE_CURRENT_SOURCE_DIR}/ateam_controls_c/src/*.rs"
  "${CMAKE_CURRENT_SOURCE_DIR}/analysis/src/*.rs"
)
file(GLOB_RECURSE CARGO_TOMLS
  "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml"
  "${CMAKE_CURRENT_SOURCE_DIR}/*/Cargo.toml"
)

add_custom_command(
  OUTPUT ${IMPORTED_LIB_PATH}
  COMMAND cargo build --workspace $<$<CONFIG:Release,RelWithDebInfo>:--release>
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS ${RUST_SOURCES} ${CARGO_TOMLS}
  COMMENT "Building Rust library with Cargo"
)
add_custom_target(
  build_rust_lib ALL
  DEPENDS ${IMPORTED_LIB_PATH}
)

add_library(ateam_controls STATIC IMPORTED GLOBAL)
set_target_properties(
  ateam_controls PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/ateam_controls_c/include"
  IMPORTED_LOCATION ${IMPORTED_LIB_PATH}
  IMPORTED_GLOBAL TRUE
)
add_dependencies(ateam_controls build_rust_lib)

# Install the library
install(
  FILES ${IMPORTED_LIB_PATH}
  DESTINATION lib
)

# Install headers
install(
  DIRECTORY ateam_controls_c/include/
  DESTINATION include
)

# Export the library so other packages can find it
ament_export_include_directories(include)
ament_export_libraries(ateam_controls_c)
ament_export_link_flags("-L${CMAKE_INSTALL_PREFIX}/lib")

enable_testing()
add_subdirectory(cpp_tests)

ament_package()
