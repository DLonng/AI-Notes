# ROS 工程安装缺少的依赖库
仅限于此 ROS 节点！

## 一、ROS 安装 sensor_msg

编写一个头文件发现没有找到 sensor_msgs 包：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/no_found_sensor_msgs.png)


查找了下博客，发现直接 apt 安装即可：

```shell
sudo apt install ros-kinetic-common-msgs
```


- 参考链接：[why no such directory "sensor_msgs"](https://answers.ros.org/question/34273/why-no-such-directory-sensor_msgs/)




