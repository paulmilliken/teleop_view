cmake_minimum_required(VERSION 2.8)
project(teleop_view)

find_package(catkin REQUIRED)
catkin_package()

find_package(Boost COMPONENTS signals)
find_package(GTK2)
find_package(OpenCV)
find_package(catkin REQUIRED camera_calibration_parsers cv_bridge image_transport message_filters nodelet rosconsole roscpp)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS}
                           ${GTK2_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)
add_definitions(-DHAVE_GTK)


# Nodelet library
add_library(teleop_view src/nodelets/window_thread.cpp src/nodelets/teleop_nodelet.cpp)
target_link_libraries(teleop_view ${catkin_LIBRARIES}
                                 ${GTK_LIBRARIES}
                                 ${GTK2_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
)
install(TARGETS teleop_view
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(teleop_view_exe src/nodes/teleop_view.cpp)
SET_TARGET_PROPERTIES(teleop_view_exe PROPERTIES OUTPUT_NAME teleop_view)
target_link_libraries(teleop_view_exe ${catkin_LIBRARIES}
					${OpenCV_LIBRARIES}
)

install(TARGETS teleop_view
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
