cmake_minimum_required(VERSION 3.12)
project(miv_rviz_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PKG_DEPENDENCIES
  rclcpp
  rviz_common
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
foreach(TARGET_PKG ${PKG_DEPENDENCIES})
  find_package(${TARGET_PKG} REQUIRED)
endforeach()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## Prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  include/miv_rviz_plugin/multiview_panel.hpp
  src/multiview_panel.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${PKG_DEPENDENCIES}
)

target_link_libraries(
  ${PROJECT_NAME}
  Qt5::Widgets
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib  
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  rviz_common
  plugin_description.xml
)

# replaces catkin_package(LIBRARIES ${PROJECT_NAME}) 
ament_export_libraries(
  ${PROJECT_NAME}
)

#https://robotics.stackexchange.com/questions/104478/rviz2-custom-panel-plugin-could-not-load-library-dlopen-error-undefined-symbo
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()