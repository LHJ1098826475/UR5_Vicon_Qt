cmake_minimum_required(VERSION 2.8.3)
project(vicon_bridge)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    dynamic_reconfigure
    geometry_msgs
    roscpp
    tf
    diagnostic_updater
)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

# Generate messages and services
add_message_files(FILES
    Marker.msg
    Markers.msg
    TfDistortInfo.msg
)

add_service_files(FILES
    viconCalibrateSegment.srv
    viconGrabPose.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

# Generate dynamic parameters
generate_dynamic_reconfigure_options(
  cfg/tf_distort.cfg
)

catkin_package(CATKIN_DEPENDS 
    dynamic_reconfigure
    geometry_msgs
    message_runtime 
    roscpp 
)

# check for platform and use the right Vicon SDK
if(CMAKE_SIZEOF_VOID_P MATCHES "8")
    set(VICON_SDK             vicon_sdk/Vicon_SDK_1.3_Linux/64-bit/)
    set(VICON_SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK})
    set(VICON_SDK_LINK_DIR    ${PROJECT_SOURCE_DIR}/${VICON_SDK})
elseif(CMAKE_SIZEOF_VOID_P MATCHES "4")
    set(VICON_SDK             vicon_sdk/Vicon_SDK_1.3_Linux/32-bit/)
    set(VICON_SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK})
    set(VICON_SDK_LINK_DIR    ${PROJECT_SOURCE_DIR}/${VICON_SDK})
else(CMAKE_SIZEOF_VOID_P MATCHES "8")
    message(FATAL_ERROR "unable to detect platform")
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
find_library(VICON_SDK_LIBRARY ViconDataStreamSDK_CPP PATHS ${VICON_SDK_LINK_DIR})

include_directories(src ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${VICON_SDK_INCLUDE_DIR})

include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(msvc_bridge src/msvc_bridge.cpp)

add_executable(vicon_bridge src/vicon_bridge.cpp)
target_link_libraries(vicon_bridge
    msvc_bridge
    ${VICON_SDK_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(vicon_bridge ${PROJECT_NAME}_gencpp)

add_executable(calibrate src/calibrate_segment.cpp)
target_link_libraries(calibrate
   ${catkin_LIBRARIES}
)
add_dependencies(calibrate ${PROJECT_NAME}_gencpp)

add_executable(tf_distort src/tf_distort.cpp)
target_link_libraries(tf_distort
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(tf_distort ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(testclient src/ViconDataStreamSDK_CPPTest.cpp)
target_link_libraries(testclient 
    ${VICON_SDK_LIBRARY}
    ${Boost_LIBRARIES}
)

add_executable(robot_localization src/robot_localization.cpp)
target_link_libraries(robot_localization
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(robot_localization ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


add_executable(listen_tf src/listen_tf.cpp)
target_link_libraries(listen_tf
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)


add_executable(beginner_tutorials src/beginner_tutorials.cpp)
target_link_libraries(beginner_tutorials
  ${catkin_LIBRARIES}
)
add_executable(convert src/convert.cpp)
target_link_libraries(convert
 ${catkin_LIBRARIES}
)

add_executable(pen_convert src/pen_convert.cpp)
target_link_libraries(pen_convert jsoncpp
 ${catkin_LIBRARIES}
)

add_executable(basetopaper src/basetopaper.cpp)
target_link_libraries(basetopaper
 ${catkin_LIBRARIES}
)

