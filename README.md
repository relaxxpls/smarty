The Better Team (TBT) welcomes you to the virtual simulation of Smarty!

```bash
$ cd catkin_ws
$ catkin make
$ source devel/setup.bash
$ roslaunch smartytbt smartytbt_gazebo.launch
$ cd src/smartytbt/scripts
$ chmod +x smarty_select_object.py
$ chmod +x smarty_object_detection.py
$ rosrun smartytbt smarty_select_user.py
```
### select color of object ###
```bash
$ rosrun smartytbt smarty_follow_user.py
```
### enjoy ###

You can also view the cameras by starting the rqt GUI!
	Procedure:
	For example with the camera view, the easiest way is to start the rqt GUI (by entering "rqt" into a terminal) then from  		the menu select Plugins->Visualization->Image View. Then select your camera feed from the pull down menu and you should 		see the video stream (you can also view this information from inside RViz, but I find this way to be more intuitive). 


Rviz Method:
launch for rviz(inorder to view the cameras)
```bash
roslaunch smartytbt load_urdf_into_rviz.launch
```
```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'
```

