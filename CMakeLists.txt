cmake_minimum_required(VERSION 3.5)

project(pointfoot_mujoco_sim)

find_package(glfw3 REQUIRED)

set(BUILD_TYPE_RELEASE TRUE)

set(MUJOCO_MODEL_PATH ${PROJECT_SOURCE_DIR}/model/robot.xml)

configure_file(config/robot_config.h.cmake.in ${CMAKE_BINARY_DIR}/mujoco/robot_config.h @ONLY)

include_directories(${CMAKE_BINARY_DIR})

include_directories(
  sdk/mujoco-2.2.0/include
  sdk/pointfoot_sdk_sim/include
  ${CMAKE_BINARY_DIR}
)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  link_directories(sdk/mujoco-2.2.0/lib/aarch64)
  link_directories(sdk/pointfoot_sdk_sim/lib/aarch64)
  install(DIRECTORY sdk/mujoco-2.2.0/lib/aarch64/. DESTINATION lib)
  install(DIRECTORY sdk/pointfoot_sdk_sim/lib/aarch64/. DESTINATION lib)
else()
  link_directories(sdk/mujoco-2.2.0/lib/amd64)
  link_directories(sdk/pointfoot_sdk_sim/lib/amd64)
  install(DIRECTORY sdk/mujoco-2.2.0/lib/amd64/. DESTINATION lib)
  install(DIRECTORY sdk/pointfoot_sdk_sim/lib/amd64/. DESTINATION lib)
endif()

add_executable(pointfoot_mujoco_sim src/simulator.cpp src/uitools.c)

add_executable(pointfoot_mujoco_vis src/visualizer.cpp src/uitools.c)

target_link_libraries(pointfoot_mujoco_sim pointfoot_sdk_sim mujoco glfw pthread rt)
target_link_libraries(pointfoot_mujoco_vis pointfoot_sdk_sim mujoco glfw pthread rt)

install(TARGETS pointfoot_mujoco_sim pointfoot_mujoco_vis DESTINATION bin)
