cmake_minimum_required(VERSION 3.28)

project(ateam_controls LANGUAGES CXX)

set(RUST_TARGET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/target)

if((CMAKE_BUILD_TYPE STREQUAL "Release") OR (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo"))
  set(IMPORTED_LIB_PATH ${RUST_TARGET_DIR}/release/libateam_controls_c.a)
else()
  set(IMPORTED_LIB_PATH ${RUST_TARGET_DIR}/debug/libateam_controls_c.a)
endif()

add_custom_command(
  OUTPUT ${IMPORTED_LIB_PATH}
  COMMAND cargo build --workspace $<$<CONFIG:Release,RelWithDebInfo>:--release>
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
add_custom_target(
  build_rust_lib ALL
  DEPENDS ${IMPORTED_LIB_PATH}
)

add_library(ateam_controls STATIC IMPORTED)
set_target_properties(
  ateam_controls PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/ateam_controls_c/include"
  IMPORTED_LOCATION ${IMPORTED_LIB_PATH}
)
add_dependencies(ateam_controls build_rust_lib)

enable_testing()
add_subdirectory(cpp_tests)
