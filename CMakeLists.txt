cmake_minimum_required(VERSION 2.8.3)
project(zeabus_imaging_sonar)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

# Determine if the BLUEVIEW_SDK_ROOT has been set
message("==================================")
if (DEFINED ENV{BLUEVIEW_SDK_ROOT})  
  message("Found Blueview SDK at:")
  message("$ENV{BLUEVIEW_SDK_ROOT}")  
else()
  message("WARNING: Can't find the BlueView SDK")
  message("Set the BLUEVIEW_SDK_ROOT environment variable in your .bashrc")
  message("if you require the syllo_blueview library")
  message("")
  return() # exit quietly if sdk not found
endif()

# Include the BLUEVIW SDK header files
include_directories($ENV{BLUEVIEW_SDK_ROOT}/include)

# Set the Blueview SDK libraries
set(BLUEVIEW_SDK_LIBS $ENV{BLUEVIEW_SDK_ROOT}/lib/libbvtsdk.so)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

message("=======>")
message("${OpenCV_INCLUDE_DIRS}")

set( LIB_NAME "sdk_blueview" )

add_message_files(
   FILES
   sonar_msg.msg 
)

add_service_files(
   FILES
   sonar_srv.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include $ENV{BLUEVIEW_SDK_ROOT}/include
  LIBRARIES ${LIB_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(open_son src/open_son.cpp)
add_executable(connect_sonar src/connect_sonar.cpp)
add_executable(record_sonar src/record_son.cpp)

target_link_libraries(open_son
  ${BLUEVIEW_SDK_LIBS}
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  opencv_highgui
)

target_link_libraries(connect_sonar
  ${BLUEVIEW_SDK_LIBS}
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  opencv_highgui
)

target_link_libraries(record_sonar
  ${BLUEVIEW_SDK_LIBS}
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  opencv_highgui
)

add_dependencies(open_son test_publish_generate_messages_cpp)
add_dependencies(connect_sonar test_publish_generate_messages_cpp)
add_dependencies(record_sonar test_publish_generate_messages_cpp)

