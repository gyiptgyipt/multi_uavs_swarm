#  MULTI UAVS SWARM SIMULATION ROS2


![UAV_show](/images/UAV_show.jpg)


[YOUTUBE_DEMO_LINK](https://www.youtube.com/watch?v=xae4pNKRpkI)

-Multi drones simulation with excutive modes

requirements

```
ros version - humble
gazebo classic - 11
```

# SETUP

### install gazebo classic 

```
sudo apt install gazebo libgazebo-dev tmuxinator ros-humble-gazebo-ros ros-humble-catch-ros2
```


### setup PX4-autopilot

```
cd ~
git clone git@github.com:PX4/PX4-Autopilot.git
cd PX4-Autopilot/
git submodule update --init --update
```
and test with
```
make px4_sitl_default gazebo
```
and you will see the Iris drone and the world


# BUILD AND INSTALL 

- go to your workspace

```
git clone git@github.com:gyiptgyipt/multi_uavs_swarm.git
```
```
cd multi_uavs_swarm
```
```
colcon build
```


# RUN 

```
cd $your workspace/multi_uavs_swarm/src/mytmux/multi_tmux
```

-In tmux folder.

run multi_tmux ( the simulation will show up)
```
tmuxinator
```

than run in new terminal

```
ros2 run px4_swarm_controller arm
```


#TODO
- switch mode 
- image request
- dynamics waypoints generator 
- obsticle avoidance

[BACKUP_PX4](https://drive.google.com/file/d/1RwTuhziePmzgBYGuAjd_7yp68xEzyOX0/view?usp=drive_link)