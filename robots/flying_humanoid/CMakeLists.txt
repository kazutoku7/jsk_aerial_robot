cmake_minimum_required(VERSION 3.0.2)
project(flying_humanoid)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  mujoco_ros_utils
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  spinal
  std_msgs
  std_srvs
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${IPOPT_INCLUDE_DIRS}
  )

add_library(flying_humanoid_robot_model src/robot_model.cpp)
target_link_libraries(flying_humanoid_robot_model ${catkin_LIBRARIES} pinocchio::pinocchio ${urdfdom_LIBRARIES})

add_executable(walk_planner src/walk_planner.cpp src/walk_planner_node.cpp)
target_link_libraries(walk_planner flying_humanoid_robot_model ${catkin_LIBRARIES} crocoddyl::crocoddyl)

add_library(flying_humanoid_static_wrench_optimizer
  src/static_wrench_optimizer.cpp
  src/surface_effect.cpp)
target_compile_options(flying_humanoid_static_wrench_optimizer PRIVATE ${IPOPT_CFLAGS_OTHER})
target_link_libraries(flying_humanoid_static_wrench_optimizer
  flying_humanoid_robot_model
  ${catkin_LIBRARIES}
  crocoddyl::crocoddyl
  ${IPOPT_LIBRARIES})

add_executable(static_wrench_optimizer src/static_wrench_optimizer_node.cpp)
target_link_libraries(static_wrench_optimizer flying_humanoid_static_wrench_optimizer)

add_library(flying_humanoid_walk_controller src/walk_controller.cpp)
target_link_libraries(flying_humanoid_walk_controller flying_humanoid_robot_model ${catkin_LIBRARIES})

install(TARGETS flying_humanoid_robot_model flying_humanoid_walk_controller walk_planner
                flying_humanoid_static_wrench_optimizer static_wrench_optimizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY config gazebo launch meshes plugins robots urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

catkin_install_python(PROGRAMS scripts/load_initial_joint_states.py scripts/spawn_static_wrench_gazebo.py
                               scripts/summarize_static_wrench_results.py
                               scripts/visualize_static_wrench_result.py
                               scripts/walk_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)
