### Gazebo 与 ROS 连接

实现 ROS 与 Gazebo 的连接，需要安装一个叫做 `gazebo_ros_pkgs`，ROS kinetic 推荐安装 Gazebo 7.x 的版本，默认全部安装会自带对应的 Gazebo 版本：

```shell
gazebo -version
```

直接 apt 安装：

```shell
sudo apt install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-msgs ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros-control
```

测试是否连接正常：

```shell
roscore
```

```shell
rosrun gazebo_ros gazebo
```

查看 topic，有 gazebo 话题则说明连接 ros 成功：

```shell
rostopic list

/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
```

参考博客：

- https://www.cnblogs.com/tanshengjiang/p/12293024.html
- https://blog.csdn.net/hitgavin/article/details/51997379)

#### RVIZ 显示 URDF 机器人模型

首先检查模型是否正常：

```shell
check_urdf *.urdf
```

使用自带 ROS 节点启动 RVIZ 查看模型：

```shell
roslaunch urdf_tutorial display.launch model:=/xx/robot_name.urdf
```

`display.launch` 如下：

```xml
<launch>

    <arg name="model" />
    <arg name="gui" default="true" />
    <arg name="rvizconfig" default="$(find urdf_tutorial)/rviz/urdf.rviz" />

    <param name="robot_description" command="$(find xacro)/xacro.py $(arg model)" />
    <param name="use_gui" value="$(arg gui)"/>

    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />

</launch>
```

#### RVIZ 显示 URDF：No transform from [xxx] to [xxx]

解决方法：

```shell
sudo apt-get install unicode
```

参考博客：

- [ROS 中遇到的问题：no transform from [left_back_wheel] to [base_link]](https://blog.csdn.net/heres_/article/details/52461596)
- [rviz 显示 urdf 模型：No transform from [base_link] to [base_footprint]](https://blog.csdn.net/maizousidemao/article/details/87621383?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param)
- [ROS No transform from [sth] to [sth]](

### Gazebo 导入 urdf 和 xacro

```xml
<param name="robot_description" command="cat '$(find robot_description)/urdf/robot.urdf'" />
```

```xml
<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find agilex_description)/urdf/xacro/gazebo/mbot_with_laser_gazebo.xacro'" /> 
```

### GazeboRosControlPlugin missing

```shell
GazeboRosControlPlugin missing <legacyModeNS> while using DefaultRobotHWSim, defaults to true
```

解决方法：

```xml
<gazebo>
  <static>false</static>

  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/mugator_manip</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    <legacyModeNS>true</legacyModeNS>
  </plugin>
</gazebo>
```

参考博客：

- https://answers.ros.org/question/292444/gazebo_ros_control-plugin-gazeboroscontrolplugin-missing-legacymodens-defaultrobothwsim/

### RVIZ 突然不能正常显示模型

如果之前一直可以，突然不行，可以重启电脑

### Gazebo 小车不走

检查轮子是否悬空了

小车轮子半径 和 base_link 的一半，要合适接地面 