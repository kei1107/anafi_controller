cmake_minimum_required(VERSION 2.8.3)
project(anafi_controller)
set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        rospy
        roscpp
        cv_bridge
        message_filters
        image_transport
        image_geometry
        std_msgs
        std_srvs
        sensor_msgs
        rosbag
        pcl_ros
        tf
        dynamic_reconfigure

        ###################
        # nodelet setting #
        ###################
        pluginlib
        nodelet
        ###################
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}

)

include_directories(
        include
        ${BOOST_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

###################
# nodelet setting #
###################
add_library(${PROJECT_NAME}
        src/nodelets/track_target_points.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
###################