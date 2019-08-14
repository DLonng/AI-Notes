# Rviz Coordinate Frames 学习笔记
Rviz 工具使用 tf 变换系统讲数据从其到达的坐标帧转换为全局参考帧。

在 Rviz 中有 2 种坐标系需要我们重点学习了解：
- Fixed Frame：固定帧
- Target Frame：目标帧

## Fixed Frame
Fixed Frame 是更为重要的帧，它是用于表示「世界」的参考坐标帧。

必须理解的是：固定帧不应该相对与 world 而发生移动，从它名字中的「固定」很容易记住这一特点。

并且如果你更改了当前的 Fixed Frame，那么 Rviz 会清楚当前的显示而不是重新进行转换。

这里介绍一个我刚开始用 Rviz 观察 Robot Model 的一个问题：

就是打开 Rviz 添加 Robot Model 后没有任何东西显示，并且 Global Status 显示警告：
```
FixedFrame：No tf data. Actual error Fixed Frame [xx] does not exist
```
这个警告就是因为 Global Options 中的 Fixed Frame 没有选择正确，我当时是选择到 base_link 后，机器人模型才显示出来。

但是之前没有学习 Rviz 的坐标系统，导致花了很长时间才解决。

## Target Frame
理解了固定帧，再来看目标帧就容易多了，目标帧就是摄像机视图的参考帧。

比如你的 Target Frame 是地图，那么你会看到机器人在地图上行驶，而如果你的 Target Frame 是机器人的底座，那么机器人将保持不动，而机器人周围的东西都会相对与机器人移动，可以简单理解为这个机器人作为你的观察目标。

如果不是很理解这两个帧的话，再看看这个问答：https://answers.ros.org/question/197740/what-does-the-fixed-frame-mean-in-rviz/