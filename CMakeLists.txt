cmake_minimum_required(VERSION 3.0.2)
project(wind2geojson)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    mavros_msgs
    std_srvs
    message_filters
    message_generation
)

find_package(PkgConfig REQUIRED)
pkg_search_module(JSONCPP jsoncpp)

find_package(Boost REQUIRED)

add_service_files(
 FILES
 start_stop_recording.srv
)

generate_messages(
    DEPENDENCIES
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_lib
    CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs std_srvs message_filters message_generation
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/geojson_handler.hpp
    include/${PROJECT_NAME}/ros_handler.hpp
)

set(${PROJECT_NAME}_SOURCES
    src/lib/ros_handler.cpp
)

add_library(${PROJECT_NAME}_lib
    ${${PROJECT_NAME}_HEADERS}
    ${${PROJECT_NAME}_SOURCES}
)

add_dependencies(${PROJECT_NAME}_lib
    ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_lib
)

target_link_libraries(${PROJECT_NAME}_lib
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${JSONCPP_LIBRARIES}
)
