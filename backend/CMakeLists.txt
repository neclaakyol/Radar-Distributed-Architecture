cmake_minimum_required(VERSION 3.22)

project(radar_backend VERSION 1.0.0 LANGUAGES CXX)

include(CTest)

option(RADAR_ENABLE_VULKAN "Enable Vulkan renderer when Vulkan, GLFW, and GLM are available" ON)

add_library(radar_core
  src/protocol.cpp
  src/replay_source.cpp
  src/serial_source.cpp
  src/sweep.cpp
  src/mti_cpu.cpp
  src/mti_gpu.cpp
  src/raster.cpp
  src/software_renderer.cpp
  src/geometry.cpp
  src/corroboration.cpp
)

target_include_directories(radar_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(radar_core PUBLIC cxx_std_20)

find_package(Threads REQUIRED)
target_link_libraries(radar_core PUBLIC Threads::Threads)

if(WIN32)
  target_compile_definitions(radar_core PUBLIC WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

if(RADAR_ENABLE_VULKAN)
  find_package(Vulkan QUIET)
  find_package(glfw3 QUIET)
  find_package(glm QUIET)
  if(Vulkan_FOUND AND glfw3_FOUND AND glm_FOUND)
    target_sources(radar_core PRIVATE src/vulkan_renderer.cpp)
    target_compile_definitions(radar_core PUBLIC RADAR_HAS_VULKAN=1)
    target_link_libraries(radar_core PUBLIC Vulkan::Vulkan)
    if(TARGET glfw)
      target_link_libraries(radar_core PUBLIC glfw)
    elseif(TARGET glfw3::glfw)
      target_link_libraries(radar_core PUBLIC glfw3::glfw)
    endif()
    if(TARGET glm::glm)
      target_link_libraries(radar_core PUBLIC glm::glm)
    endif()
  else()
    target_compile_definitions(radar_core PUBLIC RADAR_HAS_VULKAN=0)
  endif()
else()
  target_compile_definitions(radar_core PUBLIC RADAR_HAS_VULKAN=0)
endif()

add_executable(radar_app src/main.cpp)
target_link_libraries(radar_app PRIVATE radar_core)

add_executable(radar_make_replay tools/make_replay.cpp)
target_link_libraries(radar_make_replay PRIVATE radar_core)

add_executable(radar_tests tests/radar_tests.cpp)
target_link_libraries(radar_tests PRIVATE radar_core)

if(BUILD_TESTING)
  add_test(NAME radar_tests COMMAND radar_tests)
endif()
