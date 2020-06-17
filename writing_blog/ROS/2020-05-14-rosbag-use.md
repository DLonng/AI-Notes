---
title: ROS 机器人技术 - rosbag 详细使用教程
date: 2020-05-14 22:00:00
---
# ROS 机器人技术 - rosbag 详细使用教程
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

在 ROS 系统中，可以使用 bag 文件来保存和恢复系统的运行状态，比如录制雷达和相机话题的 bag 包，然后回放用来进行联合外参标定。

这里记录下我学习官方的 rosbag 教程的笔记：[ROS rosbag](http://wiki.ros.org/rosbag/Commandline)

## 一、rosbag 基本作用

rosbag 工具可以录制一个包、从一个或多个包中重新发布消息、查看一个包的基本信息、检查一个包的消息定义，基于 Python 表达式过滤一个包的消息，压缩和解压缩一个包以及重建一个包的索引。

rosbag 目前支持的命令如下：

- `record`：用指定的话题录制一个 bag 包
- `info`：显示一个 bag 包的基本信息，比如包含哪些话题
- `play`：回放一个或者多个 bag 包
- `check`：检查一个 bag 包在当前的系统中是否可以回放和迁移
- `fix`：修复 bag 包文件中的消息，以便可以在当前系统中播放
- `filter`：使用 Python 表达式过滤一个 bag 包的消息
- `compress`：压缩一个或多个 bag 包
- `decompress`：解压缩一个或多个 bag 包
- `reindex`：重新索引一个或多个损坏 bag 包

我在目前项目的使用中，用的最多的就是 record、info 以及 play 功能，先录制想要的话题包，录制完毕检查下包的信息，最后再回放作为算法的输入。

下面就详细学习下上面命令的用法。

## 二、rosbag record

使用 record 订阅指定主题并生成一个 bag 文件，其中包含有关这些主题的所有消息，常见用法如下。

用指定的话题 topic_names 来录制 bag 包：

```shell
rosbag record <topic_names>
```

比如录制 `rosout`、`tf`、`cmd_vel` 3 个话题，录制的 bag 包以时间命名：

```shell
rosbag recoed rosout tf cmd_vel
```

录制完保存 bag 包名称为 `session1 + 时间戳.bag` 格式：

```shell
rosbag record -o session1 /chatter
```

录制完保存为指定文件名 `session2_090210.bag`：

```shell
rosbag record -O session2_090210.bag /chatter
```

录制系统中所有的话题：

```shell
rosbag record -a
```

使用 `-h` 查看 record 使用方法，很多命令都可以用这个：

```shell
rosbag record -h
```



## 三、rosbag info

rosbag info 显示包文件内容的可读摘要，包括开始和结束时间，主题及其类型，消息计数、频率以及压缩统计信息，常见用法如下：

显示一个 bag 包的信息：

```shell
rosbag info name.bag
```

```shell
$ rosbag info foo.bag
path:        foo.bag
version:     2.0
duration:    1.2s
start:       Jun 17 2010 14:24:58.83 (1276809898.83)
end:         Jun 17 2010 14:25:00.01 (1276809900.01)
size:        14.2 KB
messages:    119
compression: none [1/1 chunks]
types:       geometry_msgs/Point [4a842b65f413084dc2b10fb484ea7f17]
topics:      /points   119 msgs @ 100.0 Hz : geometry_msgs/Point
```

查看常用命令：

```shell
rosbag info -h
```

输出 [YAML](https://yaml.org/) 格式的信息：

```shell
rosbag info -y name.bag
```

输出 bag 中指定域的信息，比如只显示持续时间：

```shell
rosbag info -y -k duration name.bag
```



## 四、rosbag play

rosbag play 读取一个或多个 bag 文件的内容，并以时间同步的方式回放，时间同步基于接收消息的全局时间。回放开始后，会根据相对偏移时间发布消息。

如果同时回放两个单独的 bag 文件，则根据时间戳的间隔来播放。比如我先录制一个 bag1 包，等待一个小时，然后录制另一个 bag2 包，那我在一起回放 bag1 和 bag2 时，在回放的中间会有 1 个小时的停滞期，也就是先回放 bag1，然后需要等待 1 个小时才能回放 bag2。

再回放过程中按**空格暂停**，常见用法如下：

回放单个 bag：

```shell
rosbag play record.bag
```

回放多个 bag，基于全局时间间隔播放：

```shell
rosbag play record1.bag record2.bag
```

开始播放立刻暂停，按空格继续：

```shell
rosbag play --pause record.bag
```

指定回放频率，默认 100HZ：

```shell
rosbag play --clock --hz=200 record.bag
```

循环播放：

```shell
rosbag play -l record.bag
```













## 1、保存系统状态

```shell
rosbag record -a
```
这个命令会自动以时间命名 bag 文件：
```shell
2019-06-15-19-55-35.bag
```
查看 bag 文件信息：
```shell
rosbag info 2019-06-15-19-55-35.bag
```
## 2、恢复系统状态
```shell
rosbag play 2019-06-15-19-55-35.bag
```
## 3、保存感兴趣的主题
对有些运行中的系统，当前含有的主题可能非常多，可以使用以下命令来只保存感兴趣的主题：
```shell
rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
```
以上命令将 cmd_vel 和 pose 主题保存到 subset.bag 文件中。

## 4、不精准
因为 roscord 命令受到系统时间微小变化的影响，所以回放的路径可能不精准。











http://wiki.ros.org/rosbag/Commandline

> {{ site.prompt }}

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)