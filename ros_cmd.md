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