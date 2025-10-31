## ROS CMakeList

#### CMakeList.txt是CMake的配置文件，负责目标源码的编译以及提供编译所需库。
#### 整个CMakeList的目标就是生成一个可执行文件，例如此处的ArmEeposeBridge.cpp就是源码，通过add_executable来加载。
#### 而要让其可编译，就需要target_link_libraries来链接所需的库。

- `find_package(REQUIRED)`

  - 查找并加载所需的ROS包和第三方库，确保编译时能找到这些包提供的头文件和库文件。

- `add_executable`

  - 定义可执行文件及其源码，也是CMake实现的核心与目标，让目标源码编译生成可执行文件。

- `target_include_directories`

  - 为可执行文件的源码提供.h头文件声明，保证所用的类和函数能被正确识别，不会出现未知符号报错。

- `target_link_libraries`

  - 为可执行文件链接所需的库，也就是头文件声明的类和函数的具体实现。

- `install(TARGETS … RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})`

  - 把可执行文件 `arm_eepose_bridge` 安装到包的 `bin` 目录（例如 `install/lib/armeepose/` 或 `install/bin`，依具体配置）。
  - 部署到 `install` 空间后，`rosrun/`系统路径能直接找到可执行文件。

- `install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch FILES_MATCHING PATTERN "*.launch")`

  - 把你的 `launch` 脚本安装到包的 `share/launch` 目录。  
  - 部署后 `roslaunch` 能从安装空间找到这些 .launch 文件。


## ROS package

#### package.xml是ROS的包管理文件，定义了包的基础信息和依赖关系，负责向catkin声明。


eg:

在 package.xml 里写：

```xml
<build_depend>orocos-kdl</build_depend>
```

→ 这样 rosdep install 就会自动帮你在系统里装好 liborocos-kdl-dev。

在 CMakeLists.txt 里写：

```cmake
find_package(orocos_kdl REQUIRED)
target_link_libraries(my_node ${orocos_kdl_LIBRARIES})
```

→ 这样编译器/链接器才知道去哪里找头文件和库。