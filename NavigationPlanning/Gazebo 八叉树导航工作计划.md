### 导航工作计划

1. Gazebo 小车建模 - ok
2. 测试融合与建图节点 - ok
3. 配置激光 SLAM lego_loam ？先使用小车的轮式里程计 /odom 测试 - ok
4. 保存八叉树地图，使用 octomap_server 提供的 2D 网格地图服务保存地图 - ok
5. 检查导航包需要的数据是否准备完毕 - ok
6. 检查小车的速度控制是否合适 - ok
7. 准备完毕即可开始配置导航功能包，从源码编译运行 - ok
8. 配置 move_base 参数 - ok
9. 配置 AMCL，AMCL 只能使用 2D 单线雷达，使用 pointcloud_to_laser 做个简单的转换 - ok
10. 使用提前构建的地图模拟导航，mbot 可以，自己构建的地图需要滤除地面就可以导航
11. 一边 SLAM 建图一边导航：gmapping 可以
12. 在 Gazebo 中测试导航功能，应该要需要增加一个 odom 到 map 的静态变换
13. 在小车上测试导航功能，应该要需要增加一个 odom 到 map 的静态变换