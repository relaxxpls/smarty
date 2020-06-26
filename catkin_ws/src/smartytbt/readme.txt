The Better Team (TBT) wlcomes you to the virtual simulation of Smarty!

1$ cd catkin_ws
1$ catkin make
1$ source devel/setup.bash
1$ roscore
2$ roslaunch smartytbt tbt_gazebo.launch
3$ rosrun smartytbt smarty_select_user.py					### select color ###
4$ rosrun smartytbt usertbt_teleop.py					### user control ###
5$ rosrun smartytbt smarty_follow_user.py					###    follow    ###

You can also view the cameras by starting the rqt GUI!
	Procedure:
	For example with the camera view, the easiest way is to start the rqt GUI (by entering "rqt" into a terminal) then from  		the menu select Plugins->Visualization->Image View. Then select your camera feed from the pull down menu and you should 		see the video stream (you can also view this information from inside RViz, but I find this way to be more intuitive). 

Rviz Method:
launch for rviz(inorder to view the cameras)
roslaunch smartytbt load_urdf_into_rviz.launch
