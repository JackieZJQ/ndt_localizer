cmake_minimum_required(VERSION 3.5)
project(ndt_localizer)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)



include_directories(${PROJECT_SOURCE_DIR}/include
                    ${EIGEN3_INCLUDE_DIR}
                    ${PCL_INCLUDE_DIRS})

## Build ##
add_executable(test_node ${PROJECT_SOURCE_DIR}/src/map_loader.cc)
ament_target_dependencies(test_node rclcpp sensor_msgs)
target_link_libraries(test_node ${PCL_LIBRARIES})

## Install ##
install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
