cmake_minimum_required(VERSION 2.8.3)
project(planning_benchmark)

find_package(catkin REQUIRED COMPONENTS
  exotica
)

find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planning_benchmark
  CATKIN_DEPENDS exotica
  DEPENDS system_lib
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(planning_benchmark_node src/benchmark.cpp)
target_link_libraries(planning_benchmark_node
  ${catkin_LIBRARIES}
)