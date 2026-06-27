cmake_minimum_required(VERSION 3.28)

project(ateam_controls LANGUAGES CXX)

set(RUST_TARGET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/target CACHE PATH "Where to place 'target' dir when running cargo.")

if((CMAKE_BUILD_TYPE STREQUAL "Release") OR (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo"))
  set(IMPORTED_LIB_PATH ${RUST_TARGET_DIR}/release/libateam_controls_c.a)
else()
  set(IMPORTED_LIB_PATH ${RUST_TARGET_DIR}/debug/libateam_controls_c.a)
endif()

file(GLOB RUST_LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ateam-controls/src/*.rs)
list(APPEND C_LIB_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/ateam-controls-c/include/ateam_controls/ateam_controls.h
  ${CMAKE_CURRENT_SOURCE_DIR}/ateam-controls-c/src/lib.rs
)
add_custom_command(
  OUTPUT ${IMPORTED_LIB_PATH}
  COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${RUST_TARGET_DIR} $<$<CONFIG:RelWithDebInfo,Debug>:RUSTFLAGS=-g> --unset=MAKEFLAGS --unset=MFLAGS cargo build --workspace $<$<CONFIG:Release,RelWithDebInfo>:--release>
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS ${C_LIB_SOURCES} ${RUST_LIB_SOURCES}
)
add_custom_target(
  build_rust_lib ALL
  DEPENDS ${IMPORTED_LIB_PATH}
)

add_library(ateam_controls STATIC IMPORTED)
set_target_properties(
  ateam_controls PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/ateam-controls-c/include"
  IMPORTED_LOCATION ${IMPORTED_LIB_PATH}
  IMPORTED_GLOBAL TRUE
)
add_dependencies(ateam_controls build_rust_lib)

enable_testing()
add_subdirectory(cpp_tests)
