# BoidsAgent
e谷动捕小车调试

在魏博的基础上略加修改，添加了Queue队列避障，本项目仅适用于上海交通大学prp项目，无其他用途

## 本机wsl配置
在windows上找到 C:\Users\\$user_name$ 用记事本编辑.wslconfig配置文件，如果没有就创建一个，写入如下配置：

[experimental]

autoMemoryReclaim=gradual 

networkingMode=mirrored

dnsTunneling=true 

firewall=true 

autoProxy=true 

sparseVhd=true

写入后，重启电脑，加载配置

## 通信配置
启用wsl终端，输入

sudo apt install ros-noetic-vrpn-client-ros

下载通信包

输入 roscore 启用ros服务

终端输入 roslaunch vrpn_client_ros sample.launch server:-192.168.1.100

与动捕连接
