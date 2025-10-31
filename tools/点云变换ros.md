
### 运行流程
---
调试前连上网线设置ip及机器人型号，所有终端都需要
```bash
export ROS_MASTER_URI=http://10.192.1.2:11311
export ROS_IP=10.192.1.200
export ROBOT_TYPE=SF_TRON1A
```

终端一：
启动主节点`pointfoot_hw.launch`，/home/tong/limx_ws/src/tron1-rl-deploy-ros/robot_hw/launch/display_robot.launch
```bash
cd ~/limx_ws
roslaunch robot_hw pointfoot_hw.launch
```

终端二：
获取关节角度并发布`joint_states`，/home/tong/limx_ws/tools/robot_bridge/robot_state_bridge.py
```bash
python3 robot_state_bridge.py
```

终端三：
display_robot.launch发布关节tf树，/home/tong/limx_ws/src/tron1-rl-deploy-ros/robot_hw/launch/display_robot.launch
```bash
roslaunch robot_hw display_robot.launch
```

终端四：
静态发布基底base到相机camer0的tf树
```bash
rosrun tf2_ros static_transform_publisher 0.13223 0.0222 -0.26826 0 1.063778179 0 base_Link camera0_link
```

终端五：
点云坐标变换，从camera0/depth/color到ankle，/home/tong/limx_ws/tools/ros_tf/dynamic_pointcloud_to_ankle.py
```bash
sudo apt-get install ros-noetic-tf2-sensor-msgs
python3 dynamic_pointcloud_to_ankle.py
```
需要额外安装sensor包