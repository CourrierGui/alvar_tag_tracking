cmake_minimum_required(VERSION 3.1.0)
project(alvar_tag_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  ar_track_alvar_msgs
  roscpp
  std_msgs
  tf
  tf2
  tf2_ros
  message_generation
  rqt_gui_cpp
  rqt_gui
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp REQUIRED)

find_package(Qt5Widgets REQUIRED)
catkin_python_setup()

add_message_files(
   # FILES
)

add_service_files(
   FILES
   calibration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES alvar_tag_tracking
  CATKIN_DEPENDS ar_track_alvar ar_track_alvar_msgs roscpp std_msgs tf tf2 tf2_ros rqt_gui_cpp
#  DEPENDS system_lib
)

set(rqt_calibration_src
  src/rqt_calibration/rqt_calibration.cpp
  src/rqt_calibration/new_cam_dialog.cpp
)

set(rqt_calibration_header
  include/rqt_calibration/rqt_calibration.hpp
  include/rqt_calibration/new_cam_dialog.hpp
)

set(rqt_calibration_ui
  src/rqt_calibration/rqt_calibration.ui
  src/rqt_calibration/new_cam_dialog.ui
)

qt5_wrap_cpp(rqt_calibration_header_mocs ${rqt_calibration_header})
qt5_wrap_ui(rqt_calibration_ui_h ${rqt_calibration_ui})

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_library(rqt_calibration
  ${rqt_calibration_src}
  ${rqt_calibration_header_mocs}
  ${rqt_calibration_ui_h}
)
qt5_use_modules(rqt_calibration Widgets)
target_include_directories(rqt_calibration PUBLIC "${CMAKE_SOURCE_DIR}/include" "${CMAKE_BINARY_DIR}")
target_link_libraries(rqt_calibration ${catkin_LIBRARIES} stdc++fs)
target_link_libraries     (rqt_calibration ${YAML_LIBRARIES})
target_link_libraries     (rqt_calibration ${YAML_CPP_LDFLAGS})
target_include_directories(rqt_calibration SYSTEM PUBLIC ${YAML_INCLUDE_DIRS})

add_executable(average_tf src/average_tf.cpp)
target_link_libraries(average_tf ${catkin_LIBRARIES})

add_executable(calibration_server src/calibration_server.cpp)
target_link_libraries(calibration_server ${catkin_LIBRARIES})

add_executable(calibration_client src/calibration_client.cpp)
target_link_libraries(calibration_client ${catkin_LIBRARIES})

add_executable(tf_prefixer src/tf_prefixer.cpp)
target_link_libraries(tf_prefixer ${catkin_LIBRARIES})

add_executable(new_tf_publisher src/new_tf_publisher.cpp)
target_link_libraries(new_tf_publisher ${catkin_LIBRARIES})

add_executable(new_tf_listener src/new_tf_listener.cpp)
target_link_libraries(new_tf_listener ${catkin_LIBRARIES})

install(PROGRAMS scripts/rqt_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
