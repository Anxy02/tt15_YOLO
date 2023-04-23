## olam 2D lidar ROS Drive and LaserScan  ##

### Version:2.0.11 ###
### update time:2022-01-22 ###

#### build ####
1. make catkin workspace at your ros machine

    > mkdir -p catkin_ws

2. copy 'src' to catkin_ws

    >cp src catkin_ws

3. install depend

    >rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

4. build

    >chmod -R 777 src
    >catkin_make

#### run ####

1. source to path

    >source devel/setup.bash

2. new terminal,then run roscore

    >roscore

3. connect lidar and host,lidar ip default 192.168.1.100,port default 2368

    host ip default 192.168.1.10
    these ip and port can be modified by vendor config software

4. run lidar drive at first terminal

    >roslaunch olelidar scan.launch

#### rviz view ####
1. new terminal,then run rviz

    >rosrun rviz rviz -f olelidar
    
2. at rviz,add topic olelidar/scan/LaserScan























