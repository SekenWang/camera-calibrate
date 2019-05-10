cmake_minimum_required(VERSION 2.8.3)
project(aruco_pos)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    message_generation
    geographic_msgs
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    tf2_eigen
    angles
    mavros_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3)

add_message_files(FILES
    ArUco_Pos_Att.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_NAME}/include)
#MESSAGE("Project Include Directories: ${PROJECT_SOURCE_DIR}/include")
#MESSAGE("Project Name: ${PROJECT_NAME}")

#Aruco_Dec
add_executable(aruco_pos  src/ArUco_Dec.cpp src/ArUco_Pos_Att.cpp src/lowpass_filter.cpp src/bandstop_filter.cpp) 
add_dependencies(aruco_pos ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_pos ${catkin_LIBRARIES} ${OpenCV_LIBS})



#ArUco_Test
add_executable(aruco_test  src/ArUco_Dec.cpp test/ArUco_Test.cpp)
add_dependencies(aruco_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_test ${catkin_LIBRARIES} ${OpenCV_LIBS})

#ArUco_Video_Record
add_executable(video_record  src/ArUco_Dec.cpp test/Aruco_Video_Record.cpp)
add_dependencies(video_record ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(video_record ${catkin_LIBRARIES} ${OpenCV_LIBS})
