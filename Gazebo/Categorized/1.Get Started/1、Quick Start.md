# 1、Quick Start
## 运行 Gazebo
启动 Shell 终端，键入：
```shell
gazebo
```

## 启动时加载一个机器人文件
```shell
gazebo worlds/actor.world
```
world 文件是 Gazebo 中的特殊文件，后面会介绍到，文件位于：
```
ls /usr/share/gazebo-version/worlds/
```

## Gazebo 的一点原理
Gazebo 分为 gzserver 和 gzclient 两部分，当我们每次启动 Gazebo 命令时，这两部分都会随之启动。

gzserve 执行物理更新循环和传感器数据产生，它是 Gazebo 软件的核心。

gzclient 是用 QT 写的一个用户操作接口，方便用户使用。