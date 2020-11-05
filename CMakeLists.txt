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


#add_executable(track_marker src/node/track_marker.cpp)
#target_link_libraries(track_marker ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(pc_view src/node/pc_view.cpp)
#target_link_libraries(pc_view ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(mod_quat src/node/mod_quat.cpp)
target_link_libraries(mod_quat ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(mod_quat2 src/node/mod_quat2.cpp)
target_link_libraries(mod_quat2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(mod_quat3 src/node/mod_quat3.cpp)
target_link_libraries(mod_quat3 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(show_way_point src/node/show_way_point.cpp)
target_link_libraries(show_way_point ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(way_point_to_pc src/node/way_point_to_pc.cpp)
target_link_libraries(way_point_to_pc ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(filter_param_updater src/node/filter_param_updater.cpp)
target_link_libraries(filter_param_updater ${catkin_LIBRARIES})

