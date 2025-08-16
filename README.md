#  MULTI UAVS SWARM SIMULATION ROS2

-Multi drones simulation with excutive modes

requirements
```
ros version - humble
gazebo classic - 11
```

# SETUP

### install gazebo classic 

    ```
    sudo apt install gazebo
    ```
    ```
    sudo apt install libgazebo-dev
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

    :TODO

[BACKUP_PX4](https://drive.google.com/file/d/1RwTuhziePmzgBYGuAjd_7yp68xEzyOX0/view?usp=drive_link)