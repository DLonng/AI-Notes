# 13、Recording and playing back data

在 ROS 系统中，可以使用 bag 文件来保存和恢复系统的运行状态。

## 1、保存系统状态
```
rosbag record -a
```
这个命令会自动以时间命名 bag 文件：
```
2019-06-15-19-55-35.bag
```
查看 bag 文件信息：
```
rosbag info 2019-06-15-19-55-35.bag
```
## 2、恢复系统状态
```
rosbag play 2019-06-15-19-55-35.bag
```
## 3、保存感兴趣的主题
对有些运行中的系统，当前含有的主题可能非常多，可以使用以下命令来只保存感兴趣的主题：
```
rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
```
以上命令将 cmd_vel 和 pose 主题保存到 subset.bag 文件中。

## 4、不精准
因为 roscord 命令受到系统时间微小变化的影响，所以回放的路径可能不精准。