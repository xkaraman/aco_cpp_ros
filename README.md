# labauth
Lab LASCM AUTH

# Open the simulation
'''roslaunch turtlebot_move robot_move_map.launch'''

## Localization
'''rosrun amcl amcl'''

## Visualization
'''rosrun rviz rviz'''

## Run Package
'''rosrun turtlebot_move goals.py'''

In RViz use the Publish Point button in order to publish the Points you wish the robot to calculate paths for.
When you are done, open a new terminal and publish to /calc_costs topic and empty String
'''rostopic pub /calc_costs std_msgs/String "data: ''" '''

The package should now print the Path length from each Point to another

