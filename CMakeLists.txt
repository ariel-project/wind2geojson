cmake_minimum_required(VERSION 3.0.2)
project(wind2geojson)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    mavros_msgs
    message_filters
)

find_package(PkgConfig REQUIRED)
pkg_search_module(JSONCPP jsoncpp)

find_package(Boost REQUIRED)

catkin_package( DEPENDS roscpp geometry_msgs mavros_msgs message_filters )

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

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_lib
)

target_link_libraries(${PROJECT_NAME}_lib
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${JSONCPP_LIBRARIES}
)
