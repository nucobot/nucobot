nucobot
=======

A small ROS controlled robot for Eurobot 2015 competition

Read the [wiki](https://github.com/nucobot/nucobot/wiki) to get started with the project

## Quick start:
To get a brief overview of the project you need to get through several simple steps:
+ Download the code (assuming you have installed [ROS](http://wiki.ros.org/hydro/Installation/Ubuntu))
```
cd ~/
git clone https://github.com/nucobot/nucobot
```

+ Configure the environment
```
~/nucobot/contrib/INSTALL.py
~/nucobot/contrib/INSTALL.py --deps
```
+ You might need a reboot after being added to 'dialout'
+ Launch the sumulator
```
ros_simulation   # Type this in terminal
```

## Possible issues:
+ To see the Rviz execute *roslaunch nucobot_description nucobot_rviz.launch*
+ To see the gazebo model execute *roslaunch nucobot_gazebo nucobot_world.launch*
+ **NOTE!** If gazebo fails to download models at the startup you'll have to do it manually:
```
cd <somewhere>
hg clone https://bitbucket.org/osrf/gazebo_models
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$PWD/gazebo_models
``` 
