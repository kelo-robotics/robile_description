cmake_minimum_required(VERSION 3.5.1)
project(robile_description)

if(POLICY CMP0148) 
	cmake_policy(SET CMP0148 OLD) 
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Check for ROS version
if(DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
else()
    message(FATAL_ERROR "ROS_VERSION environment variable is not set.")
endif()

message(STATUS "ROS version is: ${ROS_VERSION}")

if(${ROS_VERSION} EQUAL 1)
    message(STATUS "Configuring for ROS1")
    find_package(catkin REQUIRED)
    catkin_package(
        CATKIN_DEPENDS
    )
    
    install(DIRECTORY meshes robots urdf
            DESTINATION share/${PROJECT_NAME})

elseif(${ROS_VERSION} EQUAL 2)
    message(STATUS "Configuring for ROS2")
    find_package(ament_cmake REQUIRED)

    install(DIRECTORY meshes robots urdf gazebo
            DESTINATION share/${PROJECT_NAME})
            
    ament_package()

else()
    message(FATAL_ERROR "Unsupported ROS_VERSION. Please specify ROS_VERSION=1 or ROS_VERSION=2.")
endif()
