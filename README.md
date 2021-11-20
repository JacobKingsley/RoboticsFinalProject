# RoboticsFinalProject

To execute the program, open 8 terminal tabs and a browser window.

In all tabs, navigate to the vnc-ros directory. All commands must be run within this folder.

In the first tab, start the docker engine by running the command
```
docker-compose up --build
```

Once that has completed, in all other tabs run the following command
```
docker-compose exec ros bash
```

In the second tab, run the command
```
roscore
```

In the third tab, to run the environment simulation for the turtlebot, run
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

## If you are building the occupancy map for the first time
In the fourth tab, run the following command to make sure that gmapping can access the transformations between the different reference frames
```
roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

If this does not work, run the command
```
sudo apt install ros-melodic-turtlebot3-bringup
```
in order to install the packages necessary and then run it again.

Then in the fifth tab to start the gmapping run
```
rosrun gmapping slam_gmapping scan:=scan
```

To visualize the occupancy map built by the robot, in the sixth tab run
```
rviz
```

Navigate to your browser window and go to http://localhost:8080/vnc.html

Within the rviz window, add a map by topic (not by display type)

If you'd like to build the map by teleoperating the robot (the most efficient way), in the seventh tab run
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
And control the robot with the keyboard until the map is fully built

When you are confident in the map built, in the eight tab run
```
rosrun map_server map_saver -f ~/file_name
```
where ~/file_name is the path to and name of your output file

## If you are loading in a previously built yaml file

In the fourth tab, run 
```
rosrun map_server map_server file_name.yaml
```
where file_name.yaml is the name of your yaml file you are loading in.

## To initiate the personnel detection with the robot

In one tab, first run
```
python ImageProcessor.py
```

Note: it is important this is ran first.

Then, run
```
python Navigation.py
```