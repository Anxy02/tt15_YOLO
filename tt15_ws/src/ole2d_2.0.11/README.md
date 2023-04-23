## 欧镭2D激光雷达ROS驱动  ##

### Version:2.0.11 ###
### update time:2022-01-22 ###
#### 更新内容 ####
1. 增加组播适配功能;
	scan.launch定义local_ip为本地网卡接口;multiaddr为组播IP
	若multiaddr组播ip为空，则不加入组播。

#### 构建 ####
1. 在安装了ROS环境的ubuntu系统中创建工作区

    > mkdir -p ole2d_ws

2. 解压 'src' ROS驱动文件夹到ole2d_ws

    >cp src ole2d_ws

3. 安装 depend

    >rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
     

4. 编译
	
    >chmod -R 777 src
    >catkin_make
    注：编译前请chmod赋予src文件夹下可执行权限。

#### 运行 ####

1. 配置source源

    >source devel/setup.bash

2. 打开一个新的终端，运行roscore

    >roscore

3. 检查并连接激光雷达
	雷达默认出厂IP：192.168.1.100它将发送UDP数据包至192.168.1.10:2368
	因此，需配置本地静态IP：192.168.1.10子网掩码：255.255.255.0
    

4. 在当前配置source源的终端中，运行launch脚本

    >roslaunch olelidar scan.launch

#### rviz 可视化工具 ####
1. 打开新的终端，运行rviz

    >rviz rviz -f olelidar
    
2. 在rviz中add添加一个topic话题olelidar/scan/LaserScan























