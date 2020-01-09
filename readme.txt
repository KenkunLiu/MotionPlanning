To run the scripts:

	1. set the goal point in the variable desired_position in the file bug1.py(or bug2.py) and go_to_point.py. The default position is (-5, 1).
	2. launch the maze_world: $ roslaunch turtlebot3_gazebo turtlebot_maze.launch
	3. run the scripts of follow_wall.py and go_to_point.py in two different terminals:
		$ rosrun project_3 follow_wall.py
		$ rosrun project_3 go_to_point.py
	4. open another terminal, run the bug1.py(or bug2.py): $ rosrun project_3 bug1.py
	or $ rosrun project_3 bug2.py
