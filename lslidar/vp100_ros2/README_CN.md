NVILIDAR ROS2 驱动
nvilidar_ros2_driver 是一个全新的 ROS 2 软件包，旨在逐步成为 NVILIDAR 设备在 ROS 2 环境下的标准驱动包。

如何安装 ROS 2
请参考 ROS 2 官方安装文档：

https://index.ros.org/doc/ros2/Installation

如何创建 ROS 2 工作空间
请参考官方教程：

https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#create-a-workspace

如何构建 NVILIDAR ROS 2 软件包
1. 获取 ROS 2 源代码
1）将本项目克隆到 ROS 2 工作空间的 src 目录中：

git clone https://github.com/nvilidar/vp100_ros2.git
或：

git clone https://gitee.com/nvilidar/vp100_ros2.git


2）切换到 master 分支：
git checkout master

或者，也可以从官网下载安装源码：
http://www.nvistar.com/?jishuzhichi/xiazaizhongxin

2. 拷贝 ROS 2 代码

将下载或克隆得到的代码复制到 ROS 2 工作空间的 src 目录下。

3. 构建 ROS 2 软件包
cd vp100_ros2_ws
colcon build --symlink-install


注意：如果尚未安装 colcon，请参考：
https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon

4. ROS 2 环境配置
source ./install/setup.bash


建议将环境变量设置为永久生效：

echo "source ~/vp100_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

5. 确认 ROS 2 环境

通过以下命令确认 ROS 环境变量是否生效：

printenv | grep -i ROS

6. 串口配置
方式一：使用固定设备名 /dev/nvilidar

如果希望使用固定不变的设备名，请执行以下操作：

chmod 0777 vp100_ros2/startup/*
sudo sh vp100_ros2/startup/initenv.sh


注意：完成以上操作后，请重新插拔激光雷达设备。

方式二：使用系统默认串口设备名

如果直接使用系统分配的串口设备（如 /dev/ttyUSB0），需要为当前用户赋予串口访问权限：

假设用户名为 ubuntu：

sudo usermod -a -G dialout ubuntu
sudo reboot

ROS 参数配置
1. 支持的激光雷达型号

当前支持以下两种激光雷达：

VP100A：波特率 115200 bps

VP100L：波特率 230400 bps

接口函数定义
1. bool LidarProcess::LidarInitialialize()

初始化激光雷达，包括打开串口并与 SDK 参数信息进行同步。
初始化失败时返回 false。

2. bool LidarProcess::LidarTurnOn()

启动激光雷达扫描，使其开始输出点云数据。

3. bool LidarProcess::LidarSamplingProcess(LidarScan &scan, uint32_t timeout)

激光雷达实时数据输出接口。

LidarScan 数据结构说明：

字段	成员	说明
stamp	—	激光雷达时间戳，单位：纳秒
config	min_angle	最小扫描角度，0～2π（弧度）
	max_angle	最大扫描角度，0～2π（弧度）
	angle_increment	相邻两点之间的角度间隔
	scan_time	两次完整扫描之间的时间
	min_range	最小测距，单位：米
	max_range	最大测距，单位：米
points	angle	点对应的角度，0～2π
	range	点对应的距离，单位：米
	intensity	强度值（当灵敏度开启时有效）
4. bool LidarProcess::LidarTurnOff()

关闭激光雷达扫描。

5. void LidarProcess::LidarCloseHandle()

关闭串口或网络连接句柄。

运行 vp100_ros2
使用 launch 文件启动

通用命令格式：

ros2 launch vp100_ros2 [launch文件名].py

1. 连接激光雷达设备
ros2 launch vp100_ros2 vp100_launch.py

2. 使用 RViz 显示数据
ros2 launch vp100_ros2 vp100_launch_view.py

3. 查看扫描话题
ros2 run vp100_ros2 vp100_ros2_client


或：

ros2 topic echo /scan

nvilidar_ros2 参数说明
参数配置文件

文件路径： params/nvilidar.yaml

vp100_ros2_node:
  ros__parameters:
    serialport_name: "/dev/nvilidar"
    serialport_baud: 115200
    frame_id: "laser_frame"
    resolution_fixed: false
    auto_reconnect: true
    reversion: false
    inverted: false
    angle_min: -180.0
    angle_max: 180.0
    range_min: 0.001
    range_max: 64.0
    aim_speed: 6.0
    sampling_rate: 3
    angle_offset_change_flag: false
    angle_offset: 0.0
    ignore_array_string: ""
    log_enable_flag: true

参数含义说明
参数名	说明
serialport_baud	串口通信波特率
serialport_name	串口设备名称
ip_addr	使用 UDP 通信时的雷达 IP 地址（默认：192.168.1.200）
frame_id	ROS 中使用的坐标系名称
resolution_fixed	是否固定每圈输出点数
auto_reconnect	断开连接后是否自动重连
reversion	点云方向反转
inverted	点云上下翻转
angle_max	最大扫描角度（最大 180°）
angle_min	最小扫描角度（最小 -180°）
range_max	最大测距距离（默认 64 米）
range_min	最小测距距离
aim_speed	雷达转速（Hz）
sampling_rate	采样率（每秒 K 点数）
angle_offset_change_flag	是否启用角度偏移
angle_offset	角度偏移值
ignore_array_string	过滤角度区间，例如 "30,60,90,120" 表示过滤 30°–60° 和 90°–120° 的点
log_enable_flag	是否启用日志文件输出