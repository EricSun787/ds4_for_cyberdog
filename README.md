# ds4_for_cyberdog

DualShock 4(PS4) for CyberDog
![PS4](https://lh3.googleusercontent.com/proxy/FQ2CVubZgI1cdr4-nrCDfvE_Dtk1EZBXPaVbkcOnqZOSisofFL5xZ1EnDDVnMbAWVCzx9ILw4d_4x-cN7rSKqDKuHrESa7ofjxcbhGLhyLmY9gq1MJ-PEg8XG5YirjvV-xw)

## 功能
- 继承自[ds4_driver](https://github.com/naoki-mizuno/ds4_driver)项目，感谢**naoki-mizuno**~
- 实现了图形按键四种模式切换
-- Square ：姿态展示
-- Circle ：小跑
-- Triangle : 站起
-- Cross：趴下
- 摇杆操作与CyberDog APP相同
- 原有手柄驱动接口全支持,详见[ds4_driver](https://github.com/naoki-mizuno/ds4_driver)

## 安装及使用

### 安装功能包

本功能包驱动依赖于[naoki-mizuno/ds4drv](https://github.com/naoki-mizuno/ds4drv/tree/devel)(`devel` branch)。
安装方式如下：
1. 通过SSH连接到CyberDog
2. 在CyberDog终端中运行以下命令，安装ds4drv驱动
```console
pip3 install ds4drv
```
3. 在CyberDog中新建ROS2工作空间
```console
$ mkdir -p ros2_ws/src
```
4. 拷贝ds4_for_cyberdog到ROS2工作空间中并编译
```console
$ cd ~/ros2_ws/src/
$ git clone https://github.com/Sunshengjin/ds4_for_cyberdog.git
$ cd ..
$ colcon build
```

### 运行功能包

1. 将CyberDog连接到显示器，然后通过桌面图形化蓝牙配对DS4手柄
2. 通过SSH终端连接到CyberDog，运行功能包
```console
$ source ~/ros2_ws/install/setup.bash
$ roslaunch ds4_for_cyberdog ds4_cyberdog.launch.xml
```
3.此时即可遥控机器人

## TODO & Known Issue
1. 更简便的配置蓝牙手柄连接
2. 蓝牙手柄配对CyberDog后，下次启动无法自动配对
3. 在使用手柄遥控前，需使用CyberDog APP点击开始遥控，进入遥控模式，否则无法遥控
4. 将手柄功能加入自启动service
5. 添加更多功能
-- 手柄灯效与CyberDog模式联动
-- 手柄震动联动
-- ...
