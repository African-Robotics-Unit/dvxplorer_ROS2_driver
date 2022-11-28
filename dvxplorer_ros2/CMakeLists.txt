cmake_minimum_required(VERSION 3.5)
project(dvxplorer_ros2)

set(DVS_INCLUDE_PATH /usr/include)
#set(DVS_LIBRARY libcaer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dvxplorer_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(libcaer REQUIRED)

add_executable(${PROJECT_NAME}_node src/dvxplorer_ros2_node.cpp src/dvxplorer_ros2.cpp)

ament_target_dependencies(${PROJECT_NAME}_node 
  "rclcpp"
  "image_transport"
  "sensor_msgs"
  "builtin_interfaces"
  "dvxplorer_interfaces"
  "camera_info_manager"
  "image_transport"
  )

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${DVS_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_node
  #${DVS_LIBRARY}
  ${Boost_LIBRARIES}
  libcaer::caer
  )

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

  
  

ament_package()
