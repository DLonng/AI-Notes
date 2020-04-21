# 如何在 ROS 中使用八叉树 Octomap 库？

 ## 一、Octomap 项目地址

- [Octomap](https://octomap.github.io/)
- [http://octomap.github.io/octomap/doc/](http://octomap.github.io/octomap/doc/)

## 二、Octomap 安装

### 2.1 ROS 功能包安装

```shell
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server
```

或者：

```
sudo apt-get install ros-kinetic-octomap*
```

### 2.2 源码编译安装

获取源码：

```shell
git clone https://github.com/OctoMap/octomap
```

CMake 编译：

```shell
cd octomap
mkdir build
cd build
cmake ..
make
```

编译通过即可安装：

```shell
sudo make install
```

卸载方法，在 `build` 目录下执行：

```shell
sudo make uninstall
```

补充，如果没有可视化程序 octovis，可独立安装：

```shell
sudo apt-get install octovis
```



## 二、Rviz 可视化 Octomap





## 三、编程使用 Octomap

### 3.1 pcd 点云转 Octomap





### 3.2 给 Octomap 加上颜色





### 3.3 Octomap 建图

Octomap 建图就是利用位姿信息将多帧 Octomap 地图拼接成一个全局地图。



### 3.4 占有率和概率更新



## 参考博客

- [SLAM拾萃(1)：octomap](https://www.cnblogs.com/gaoxiang12/p/5041142.html)
- [https://wiki.ros.org/octomap](https://wiki.ros.org/octomap)
- [使用octomap_server将点云地图转化为八叉树地图和占据栅格地图](https://blog.csdn.net/sylin211/article/details/93743724)
- [Octomap 在ROS环境下实时显示](https://blog.csdn.net/crp997576280/article/details/74605766)
- [视觉SLAM笔记（64） 八叉树地图](https://blog.csdn.net/qq_32618327/article/details/103215769?depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-2&utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-2)
- [https://blog.csdn.net/DJ_Dreamaker/article/details/79834954](https://blog.csdn.net/DJ_Dreamaker/article/details/79834954)
- [https://blog.csdn.net/weixin_39123145/article/details/82219968](https://blog.csdn.net/weixin_39123145/article/details/82219968)

