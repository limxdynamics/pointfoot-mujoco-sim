cmake_minimum_required(VERSION 3.5)

project(pointfoot_mujoco_sim)

find_package(glfw3 REQUIRED)

set(BUILD_TYPE_RELEASE TRUE)

set(MUJOCO_MODEL_PATH ${PROJECT_SOURCE_DIR}/robot-description/pointfoot/$ENV{ROBOT_TYPE}/xml/robot.xml)

configure_file(config/robot_config.h.cmake.in ${CMAKE_BINARY_DIR}/mujoco/robot_config.h @ONLY)

include_directories(${CMAKE_BINARY_DIR})

include_directories(
  sdk/mujoco-2.2.0/include
  sdk/limxsdk_sim/include
  ${CMAKE_BINARY_DIR}
)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  link_directories(sdk/mujoco-2.2.0/lib/aarch64)
  link_directories(sdk/limxsdk_sim/lib/aarch64)
  install(DIRECTORY sdk/mujoco-2.2.0/lib/aarch64/. DESTINATION lib)
  install(DIRECTORY sdk/limxsdk_sim/lib/aarch64/. DESTINATION lib)
else()
  link_directories(sdk/mujoco-2.2.0/lib/amd64)
  link_directories(sdk/limxsdk_sim/lib/amd64)
  install(DIRECTORY sdk/mujoco-2.2.0/lib/amd64/. DESTINATION lib)
  install(DIRECTORY sdk/limxsdk_sim/lib/amd64/. DESTINATION lib)
endif()

add_executable(simulator src/simulator.cpp src/uitools.c)

target_link_libraries(simulator limxsdk_sim mujoco glfw pthread rt)

install(TARGETS simulator DESTINATION bin)
