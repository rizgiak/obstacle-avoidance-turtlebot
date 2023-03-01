# Obstacle Avoidance using Turtlebot3

Tested in Notebook Intel Core i7 (32GB) NVidia Quadro P620 with installed Ubuntu 20.04 and ROS Noetic.
Code based on Python 3.

First, create environment map.
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Generate `map.pgm` and `map.yaml` as in [this](http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) tutorial.

Make sure that navigation can be performed as in [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#perform-navigation).

If both of the step can be finished well, run the python code to create obstacle avoidance movement.
