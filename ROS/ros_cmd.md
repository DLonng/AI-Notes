## ROS 常用命令整理
### roscore
启动 ros 系统：
```
roscore
```
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
列出包的依赖：
```
rospack depends1 beginner_tutorials
```
递归显示所有依赖：
```
rospack depends beginner_tutorials
```
### rosnode
rosnode 列出当前正在运行中的节点信息：
```
rosnode list
```
使用 info 来列出指定的节点信息：
```
rosnode info /rosout
```
### rostopic
rostopic echo 得到 ROS 帆布的主题中的数据：
```
rostopic echo /turtle1/cmd_vel
```
rostopic list 返回一系列当前订阅和发布的主题：
```
rostopic list -h
```
rostopic type 返回任何一个已经发布主题的消息类型：
```
rostopic type /turtle1/cmd_vel
```
发布数据到当前的广告的主题：
```
rostopic pub [topic] [msg_type] [args]
```
```
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
rostopic 显示当前数据发布的速率：
```
rostopic hz /turtle1/pose
```
### rosls
类似 ls 命令，不多说了，都懂。
### catkin_make
编译 catkin 项目：
```
cd ~/catkin_ws
catkin_make
```
### rosrun
使用包和节点名称直接运行一个节点：
```
rosrun turtlesim turtlesim_node
```
### rosservice
rosservice 有很多用于 service 服务的命令：
```
rosservice list         print information about active services
rosservice call         call the service with the provided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri
```
### rosparam
rosparam 可以存储和修改参数服务器的数据，包含的命令有：
```
rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           dump parameters to file
rosparam delete         delete parameter
rosparam list           list parameter names
```
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