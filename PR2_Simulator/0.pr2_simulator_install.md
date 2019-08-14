# PR2_Simulator
学习完 ROS Wiki 的基础教程后，我下一步选择学习 PR2 模拟器：http://wiki.ros.org/pr2_simulator/Tutorials

于是就按照官网找到这个教程，但是要先安装 PR2 和 gazebo_ros 包才可以，这里记录下自己的安装过程和踩过的坑。
## 安装 gazebo_ros_pkgs
运行教程中的第一行命令：
```
$ roslaunch gazebo_ros empty_world.launch
[empty_world.launch] is neither a launch file in package [gazebo_ros] nor is [gazebo_ros] a launch file name
The traceback for the exception was written to the log file
```
这个原因是 gazebo_ros_pkgs 未安装，那就按照 gazebo 给的安装[教程](http://gazebosim.org/tutorials?tut=ros_installing)来安装它：
```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
可是期间出现了依赖问题，我就换 Aptitude 安装来自动安装依赖关系：
```
sudo apt install aptitude
```
然后重新安装 gazebo-ros 包：
```
sudo aptitude install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
安装完后，又出现 rosrun 命名找不到，我猜想应该是安装过程中 aptitude 勿删除了，不过没关系，重新安装下 ROS_Base 就行了：
```
sudo apt-get install ros-kinetic-ros-base
```
测试下：
```
source ~/catkin_ws/devel/setup.bash
roscore &
rosrun gazebo_ros gazebo
```
出现 GUI 界面即安装成功。
## 安装 PR2 模拟器
因为 zsh 不支持通配符安装，所以要先在 .zshrc 末尾加上 `setopt nomatch`，然后就可以使用下面的方式安装 PR2 了：
```
sudo apt-get install ros-kinetic-pr2-*
```
安装需要挺久，建议找个好点的网速下载。
## 测试 PR2
先启动 roscore：
```
roscore
```
一次性加载 PR2 到 Gazebo：
```
roslaunch pr2_gazebo pr2_empty_world.launch
```
然后启动键盘控制节点，测试是否能够控制 PR2 移动和旋转：
```
roslaunch pr2_teleop teleop_keyboard.launch
```
可以移动和旋转，搞定了！