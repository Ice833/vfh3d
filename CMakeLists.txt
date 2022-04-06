cmake_minimum_required(VERSION 2.8.3)
project(vfh_rover)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_ros
  octomap_msgs
  PCL
  pcl_conversions
)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

## Uncomment this if the package has a setup.py
# catkin_python_setup()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES vfh_rover
 CATKIN_DEPENDS roscpp
 DEPENDS system_lib PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

set(SRC
  "src/PolarHistogram.cpp"
  "src/VFHistogram.cpp"
  "src/OctomapProcessing.cpp"
  "src/util.cpp"
)

add_executable(${PROJECT_NAME}_node src/vfh_rover_node.cpp ${SRC})
target_link_libraries(${PROJECT_NAME}_node ${LINK_LIBS})

add_executable(drone_node src/drone_node.cpp ${SRC})
target_link_libraries(drone_node ${LINK_LIBS})

# add_executable(cloud_node src/cloudProcessing.cpp)
# target_link_libraries(cloud_node ${LINK_LIBS})

add_executable(${PROJECT_NAME}_testPC src/SingleVoxTest.cpp ${SRC})
target_link_libraries(${PROJECT_NAME}_testPC ${LINK_LIBS})
