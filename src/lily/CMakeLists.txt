cmake_minimum_required(VERSION 3.5)
project(lily)

set(CMAKE_CXX_STANDARD 17)

# no warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# find Boost
find_package(Boost COMPONENTS program_options filesystem REQUIRED)

# ros
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    tf2
    pcl_conversions
    pcl_ros
    dynamic_reconfigure
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/dynamic.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs tf2 pcl_conversions pcl_ros dynamic_reconfigure
    DEPENDS OpenCV PCL
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

aux_source_directory(./src SRC)
add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
)
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
