nucobot
=======

A small ROS controlled robot for Eurobot 2015 competition

Read the wiki to get started with the project

+ To see the Rviz execute *roslaunch nucobot_description nucobot_rviz.launch*
+ To see the gazebo model execute *roslaunch nucobot_gazebo nucobot_world.launch*
+ **NOTE!** If gazebo fails to download models at the startup you'll have to do it manually:
```
cd <somewhere>
hg clone https://bitbucket.org/osrf/gazebo_models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$PWD/gazebo_models
``` 
