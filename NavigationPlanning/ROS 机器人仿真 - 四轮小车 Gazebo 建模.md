## 编写小车 URDF、xacro 模型

#### Gazebo 小车建模

1. 学习古月居 mbot 仿真
2. 创建仿真功能包
3. 根据用户手册对小车建模 URDF，xacro，sdf？
4. 测试模型

#### TF 坐标系

- 蓝：X
- 绿：Y
- 红：Z

每个 link 的坐标系需要搞清楚！

#### Axis

旋转轴，type 标签里面反复提到的旋转轴就是这个标签，它指示了绕什么轴进行旋转，例如绕**父节点的 z 轴旋转**，那么该标签的值就为 xyz=“0 0 1”

#### Gazebo 中不显示模型或者不完全显示

需要为 link 加上碰撞检测惯性系数：

```xml
<collision>
    <geometry>
    	<cylinder length="${length_wheel}" radius="${radius_wheel}"/>
    </geometry>
</collision>
<xacro:default_inertial mass="1"/>
```

#### 添加 stere camera 插件

```xml
<!-- camera_link -->
  <gazebo reference="zed_camera">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="zed_camera">
    <sensor type="multicamera" name="stereo_camera">
      <update_rate>30.0</update_rate>
      <camera name="left">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>672</width>
          <height>376</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <camera name="right">
        <pose>0 -0.07 0 0 0 0</pose>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>672</width>
          <height>376</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>

      <plugin name="stereo_camera_controller" filename="libgazebo_ros_multicamera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>left_camera</frameName>
        <rightFrameName>right_camera</rightFrameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
```



参考博客：

- http://gazebosim.org/tutorials?tut=ros_gzplugins#Camera

#### 添加 Robosense 激光雷达插件

```xml
<!-- Gazebo requires the rslidar_gazebo_plugins package -->
  <gazebo reference="rslidar">
    <sensor type="ray" name="rslidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>20</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>600</samples>
            <resolution>1</resolution>
            <min_angle>0.4</min_angle>
            <max_angle>150.0</max_angle>
          </horizontal>

          <vertical>
            <samples>16</samples>
            <resolution>1</resolution>
            <min_angle>-${15.0*3.1415926/180.0}</min_angle>
            <max_angle> ${15.0*3.1415926/180.0}</max_angle>
          </vertical>

        </scan>

        <range>
          <min>0.4</min>
          <max>3.1415926</max>
          <resolution>0.001</resolution>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.0</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_robosense_laser.so">
        <topicName>/rslidar_points</topicName>
        <frameName>rslidar</frameName>
        <min_range>-3.1415926</min_range>
        <max_range>3.1415926</max_range>
        <gaussianNoise>0.008</gaussianNoise>
      </plugin>
    </sensor>
  </gazebo>
```



参考博客：

- https://blog.csdn.net/weixin_44172961/article/details/103493390
- https://github.com/tomlogan501/robosense_simulator



