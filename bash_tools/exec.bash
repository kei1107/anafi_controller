DIR_NAME=`date +%Y%m%d_%H-%M-%S`
gnome-terminal --tab -- bash -c "roslaunch tello_controller tello_position_estimator.launch; bash"
gnome-terminal --tab -- bash -c "roslaunch tello_controller uav_operator.launch; bash"
gnome-terminal --tab -- bash -c "mkdir $DIR_NAME; cd $DIR_NAME; rosrun image_view image_saver __name:=image_saver _save_all_image:=false  image:=/tello/image_raw; bash"
