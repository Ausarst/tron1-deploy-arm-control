
## 参考资料
- 官方文档：https://cwjgfm21di.feishu.cn/wiki/VJA7wXBQTiwaUYksZyDcu9T2n6f
- 文档中心：https://support.limxdynamics.com/docs-center
  - 之前是用的移动操作拓展套件部署，包含双足带机械臂的14维policy部署，现在这个包是基于`tron1-rl-deploy-ros`做的，并且添加了手柄控制机械臂的功能。
  - 移动操作拓展套件文档主要包含如何真机部署。
- Github库：https://github.com/limxdynamics /
https://github.com/limxdynamics/tron1-rl-deploy-ros
  - 也支持`ros2`/`python`等版本，详见Github库说明。 

## limx_ws大致框架
- limxsdk_lowlevel
  - 机器人底层SDK接口，官方接口文档：https://cwjgfm21di.feishu.cn/wiki/Rj6mwC4ewiDup7kzZKFcT8fGnIb
- tron1-rl-deploy-ros-master
  - robot_hw
    - 硬件接口节点 PointfootHW
  - robot_controller
    - 控制器插件，包含不同底盘的控制器，目前主要用到的机器人类型是Solofoot平足
  - armeepose
    - 手柄控制的桥接节点，代码说明参照随行文档`1009.pptx`
  - airbot-sdk-2.9
    - 机械臂SDK
- robot_description
  - 机器人URDF模型等描述文件
- robot-joystick
  - 手柄输入节点
- limx_ws/tools
  - 点云变换等工具脚本，参考`点云变换ros.md`

## 部署环境配置
- 主要参考：https://github.com/limxdynamics/tron1-rl-deploy-ros
- 安装ROS Noetic：我们推荐在Ubuntu 20.04操作系统上建立基于ROS Noetic的算法开发环境。ROS提供了一系列工具和库，如核心库、通信库和仿真工具（如Gazebo），极大地便利了机器人算法的开发、测试和部署。这些资源为用户提供了一个丰富而完整的算法开发环境。ROS Noetic 安装请参考文档：https://wiki.ros.org/noetic/Installation/Ubuntu ，选择“ros-noetic-desktop-full”进行安装。ROS Noetic 安装完成后，Bash终端输入以下Shell命令，安装开发环境所依赖的库：

    ```bash
    sudo apt-get update
    sudo apt install ros-noetic-urdf \
                 ros-noetic-kdl-parser \
                 ros-noetic-urdf-parser-plugin \
                 ros-noetic-hardware-interface \
                 ros-noetic-controller-manager \
                 ros-noetic-controller-interface \
                 ros-noetic-controller-manager-msgs \
                 ros-noetic-control-msgs \
                 ros-noetic-ros-control \
                 ros-noetic-gazebo-* \
                 ros-noetic-robot-state-* \
                 ros-noetic-joint-state-* \
                 ros-noetic-rqt-gui \
                 ros-noetic-rqt-controller-manager \
                 ros-noetic-plotjuggler* \
                 cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                 python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
    ```

    

- 可选安装onnxruntime依赖，下载连接：https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0  。请您根据自己的操作系统和平台选择合适版本下载。如在Ubuntu 20.04 x86_64，请按下面步骤进行安装：
  
    ```Bash
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
    
    tar xvf onnxruntime-linux-x64-1.10.0.tgz
    
    sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
    sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
    ```

## 手柄控制部分

### 启动流程

- 进入工作空间：
```bash
cd ~/limx_ws
catkin_make install
```

- 配置环境变量 `ROBOT_TYPE` 为机器人型号，然后编译、运行 ros-master：
```bash
echo 'export ROBOT_TYPE=SF_TRON1A' >> ~/.bashrc && source ~/.bashrc
```

- 配置环境变量 `RL_TYPE` 训练方法，然后编译、运行 rl-deploy：
```bash
echo 'export RL_TYPE=isaacgym' >> ~/.bashrc && source ~/.bashrc
```

#### 利用仿真主节点启动：

- 终端 1：启动主节点
```bash
source install/setup.bash
roslaunch robot_hw pointfoot_hw_sim.launch
```
- 终端 2：启动桥接节点
```bash
source install/setup.bash
roslaunch armeepose arm_eepose_bridge.launch
```
- 此时运行仿真手柄：
```bash
./robot-joystick/robot-joystick
```

### 改动
- PointfootHW.h:
  - 新增成员 `ros::Publisher ee_rc_cmd_delta_pub_;`
- PointfootHW.cpp:
  - 引入 `std_msgs/Float32MultiArray.h`
  - 在 `init()` 中 `advertise` 了 `/EEPose_cmd_rc`
  - 在摇杆回调中构造并发布 `Float32MultiArray`，映射如下：
    - 平移：
      - 未按 R2 时，L/R→Y 轴，U/D→X 轴；按住 L2 时 U/D→Z 轴、X 轴置 0
    - 旋转：
      - Y/A 控制 Pitch，X/B 控制 Roll；按住 L2 切换 X/B 控制 Yaw、Roll 置 0
    - 夹爪：R1 作为开合信号（1=开请求）
- CMakeLists.txt 和 package.xml：添加 `std_msgs` 依赖



现在在硬件层的 `PointfootHW` 里已经补全 `/EEPose_cmd_rc` 机械臂末端控制的发布逻辑，仿真/真实手柄数据来源都是厂家 SDK 的 `subscribeSensorJoy(...)` 回调，`pointfoot_hw.launch` 或 `pointfoot_hw_sim.launch` 主节点正常运行时就能拿到手柄输入，随后我们把手柄→末端位姿增量映射后按 30Hz 发布到 `/EEPose_cmd_rc`。桥接节点只需订阅这个话题即可驱动机械臂 IK。


- 添加机械臂SDK和桥接节点armeepose到工作空间（ros-master 根目录，和 robot_hw 同级）

  - limx_ws/src/tron1-rl-deploy-ros-master/airbot-sdk-2.9
    - 从 airbot-sdk-2.9 完整拷贝整个文件夹
  - limx_ws/src/tron1-rl-deploy-ros-master/armeepose



### 1) PointfootHW.h
---
在发布器成员处新增一个用于末端位姿增量的发布器，并配上中文注释：

```cpp
// 原有
ros::Publisher cmd_vel_pub_;

// 新增：机械臂末端位姿“增量”命令发布器，话题名 /EEPose_cmd_rc
// 数据格式为 std_msgs::Float32MultiArray，长度 7：
//   [0..2] = [dx, dy, dz]（m），[3..5] = [dR, dP, dY]（rad），[6] = gripper（0~1）
ros::Publisher ee_rc_cmd_delta_pub_;
```

### 2) PointfootHW.cpp
---
- 头部包含：引入 Float32MultiArray，并加中文注释

```cpp
#include "robot_hw/PointfootHW.h"
// 新增依赖：用于发布机械臂末端位姿“增量”指令数组（Float32MultiArray）
#include "std_msgs/Float32MultiArray.h"
```

- 在 init() 里，紧跟 /cmd_vel 之后，新增 advertise /EEPose_cmd_rc，并写明格式说明

```cpp
// 发布底盘速度指令话题 /cmd_vel（原有逻辑）
cmd_vel_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
// 新增：发布机械臂末端位姿“增量”话题 /EEPose_cmd_rc，供桥接节点订阅
// 数据格式固定为长度 7:
//   [0..2] = [dx, dy, dz]（米），
//   [3..5] = [dR, dP, dY]（弧度，按 roll/pitch/yaw 顺序），
//   [6]    = gripper（0~1，开合请求/占空值；桥接节点可自行解释）
ee_rc_cmd_delta_pub_ = root_nh.advertise<std_msgs::Float32MultiArray>("/EEPose_cmd_rc", 10);
```

- 在 subscribeSensorJoy 的 30Hz 发布节流块内，原有发布 /cmd_vel 后，新增“构造并发布 /EEPose_cmd_rc”的映射代码（含详细注释）：

```cpp
// 新增：构造并发布机械臂末端“位姿增量”命令 [dx, dy, dz, dR, dP, dY, gripper]
//  - 单位：平移 m，旋转 rad；
//  - 步长：dx,dy,dz = 0.003；dR,dP,dY = 0.01；
//  - 频率：与 /cmd_vel 同步限制为 30Hz；
//  - 订阅方：arm_eepose_bridge 节点会将该增量转为 IK/Jacobian 目标；
std_msgs::Float32MultiArray ee_rc_cmd_delta_msg;
ee_rc_cmd_delta_msg.data.assign(7, 0.0f);

// 工具函数：安全获取手柄按钮状态（不存在时返回 0）
auto btn = [&](const std::string &name) -> float {
  auto it = joystick_btn_map_.find(name);
  return (it != joystick_btn_map_.end()) ? static_cast<float>(msg->buttons[it->second]) : 0.0f;
};

float U = btn("U"), D = btn("D"), L = btn("L"), R = btn("R");
float L2 = btn("L2"), R2 = btn("R2"), R1 = btn("R1");
float X = btn("X"), B = btn("B"), Y = btn("Y"), A = btn("A");

// 平移映射：
//   - 未按 R2：
//       L/R → dy（左正右负），U/D → dx（上正下负）
//   - 按住 L2：U/D 改为 dz，dx 置 0，便于水平/竖直平移切换；
//   - 按住 R2：屏蔽位移（保留给其他用法，如 IMU 校准）；
if (R2 == 0.0f) {
  ee_rc_cmd_delta_msg.data[1] = 0.003f * L - 0.003f * R; // dy
  if (L2 == 1.0f) {
    ee_rc_cmd_delta_msg.data[0] = 0.0f;                        // dx
    ee_rc_cmd_delta_msg.data[2] = 0.003f * U - 0.003f * D;     // dz
  } else {
    ee_rc_cmd_delta_msg.data[0] = 0.003f * U - 0.003f * D;     // dx
    ee_rc_cmd_delta_msg.data[2] = 0.0f;                        // dz
  }
}

// 旋转映射：
//   - Y/A → dP（俯仰），X/B → dR（横滚）；
//   - 按住 L2：X/B 改为控制 dY（偏航），并将 dR 置 0；
ee_rc_cmd_delta_msg.data[4] = 0.01f * Y - 0.01f * A; // dP
if (L2 == 1.0f) {
  ee_rc_cmd_delta_msg.data[3] = 0.0f;                   // dR
  ee_rc_cmd_delta_msg.data[5] = 0.01f * X - 0.01f * B;  // dY
} else {
  ee_rc_cmd_delta_msg.data[3] = 0.01f * X - 0.01f * B;  // dR
  ee_rc_cmd_delta_msg.data[5] = 0.0f;                   // dY
}

// 夹爪：R1 作为开合请求（1=打开；0=关闭/保持），桥接节点内部可按需实现防抖或切换逻辑；
ee_rc_cmd_delta_msg.data[6] = R1;

// 发布给桥接节点
ee_rc_cmd_delta_pub_.publish(ee_rc_cmd_delta_msg);
```

### 3) CMakeLists.txt
---
在 `find_package(catkin REQUIRED COMPONENTS ...)` 和 `catkin_package(CATKIN_DEPENDS ...)` 中加入 `std_msgs`：

```cmake
find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  std_msgs        # 新增
  limxsdk_lowlevel
  robot_common
  robot_description
  hardware_interface
  controller_manager
  urdf
)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  std_msgs        # 新增
  limxsdk_lowlevel
  robot_common
  robot_description
  hardware_interface
  controller_manager
  urdf
)
```

### 4) package.xml
---
加入对 `std_msgs` 的依赖：

```xml
<depend>roscpp</depend>
<depend>std_msgs</depend> <!-- 新增 -->
<depend>controller_interface</depend>
...
```


