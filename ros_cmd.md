## ROS 常用命令整理
### rosed
用默认编辑器来编辑文件：
```
rosed roscpp Logger.msg
```
### roscd
类似 Shell 中的 cd 切换路径：
```
roscd beginner_tutorials
```
### rosmsg
显示 msg 的类型：
```
rosmsg show Num
```
### roscp
从一个包中拷贝文件到另一个包：
```
roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```
### rossrv
显示 service 的类型：
```
rossrv show beginner_tutorials/AddTwoInts
```
### rospack
### rosls
### catkin_make
### rosmake
### rosrun
### rosbag
记录系统状态：
```
rosbag record -a
```
查看记录文件：
```
rosbag info 2019-06-15-19-55-35.bag
```
回放系统记录文件：
```
rosbag play 2019-06-15-19-55-35.bag
```
记录感兴趣的主题，存储再 subset.bag 文件中：
```
rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
```
### roswtf
当 ROS 系统出现错误时，可以用这个命令检查错误：
```
roswtf
```