

# maze_escape
https://github.com/sbeen1840/maze_escape/assets/108644811/8f205f22-8171-42be-89f3-7f9720a7ff94


## Key Features
```
- Real-time maze navigation using sensor data
- Obstacle avoidance and safe maneuvering
- Adjustable speed and turning control
- Automatic path correction and decision-making
- Command line interface for monitoring and debugging
```

## Requirements
```
- ROS (Ubuntu 20.04 Noetic)
- Turtlebot3
- Python 3
```

## How To Start

```bash
# Clone this repository
$ git clone git clone https://github.com/sbeen1840/maze_escape.git

# Go into the repository
$ cd maze_escape

# Install dependencies
$ rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y

# Build the package
$ catkin_make

# To set up the working environment
$ source devel/setup.bash

# Launch the maze_escape node
$ roslaunch maze_escape maze_escape.launch
```
## How to use
```
1.
Monitor the robot's movement:
The node will start running and display information about the robot's sensor readings and movement decisions. You can observe the robot's behavior and make adjustments if needed.

2.
Customize the behavior:
You can modify the maze_escape.py file to tweak the robot's navigation behavior, such as changing the obstacle detection thresholds, adjusting turning angles, or modifying the speed.

3.
Terminate the program:
To stop the robot and exit the program, press Ctrl+C in the terminal where the launch command was executed.
```
---
> Tistory [sbeen1840](https://sbeen1840.tistory.com/) &nbsp;&middot;&nbsp;
> GitHub [sbeen1840](https://github.com/sbeen1840) &nbsp;&middot;&nbsp;
> Velog [sbeen1840](https://velog.io/@sbeen1840)


