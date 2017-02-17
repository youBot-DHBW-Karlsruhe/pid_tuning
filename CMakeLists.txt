cmake_minimum_required(VERSION 2.8.3)
project(pid_tuning)

find_package(catkin REQUIRED COMPONENTS
  brics_actuator
  tf_conversions
  roscpp
  ik_solver_service
  brics_actuator
  trajectory_msgs
  rpg_youbot_common
  torque_control
  trajectory_generator
  message_generation
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    torque_control
    brics_actuator
    trajectory_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)

add_executable(joint_tester src/joint_tester.cpp)
add_dependencies(joint_tester ${catkin_EXPORTED_TARGETS})

target_link_libraries(joint_tester
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(cs_tester src/cs_tester.cpp)
add_dependencies(cs_tester ${catkin_EXPORTED_TARGETS})

target_link_libraries(cs_tester
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

#add_executable(circle_traj2 src/circle.cpp)
#add_dependencies(circle_traj2 ${catkin_EXPORTED_TARGETS})

#target_link_libraries(circle_traj2
#  ${catkin_LIBRARIES}
#  ${Eigen_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

