DIR_NAME=`date +%Y%m%d_%H-%M-%S`
gnome-terminal --tab -- bash -c "roslaunch tello_controller tello_position_estimator.launch; bash"
gnome-terminal --tab -- bash -c "roslaunch tello_controller uav_operator.launch; bash"
gnome-terminal --tab -- bash -c "mkdir $DIR_NAME; cd $DIR_NAME; rosrun image_view image_saver __name:=image_saver _save_all_image:=false  image:=/tello/image_raw; bash"
rosbag record -O ${DIR_NAME}.bag -b 0 -e "/tf|/tf_static|/camera11/color/image_rect|/camera11/color/camera_info|/camera12/color/image_rect|/camear12/color/camera_info|/tello/image_raw|/tello/camera_info|/camera(.*)/points_centroid|/image_saver/save"
