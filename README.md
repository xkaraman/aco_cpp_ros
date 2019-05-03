# labauth
Lab LASCM AUTH

# Open the simulation
`roslaunch turtlebot_move robot_move_map.launch`

## Visualization
### Replace ~/auth_ws/src/ with your_workspace
`rosrun rviz rviz -d ~/auth_ws/src/turtlebot_move/launch/cfg/rviz.rviz
`

In RViz use the Publish Point button in order to publish the Points you wish the robot to calculate paths for.

RViz plugin buttons are used to remove points, calculate all possible paths between current points and find the best path for robot to move along all the points.

The plugin should now show the Paths from each Point to another through a MarkerArray(topic:=turtlebot_move/all_paths) and the Best Path to follow(topic:=turtlebot_move/best_path).
