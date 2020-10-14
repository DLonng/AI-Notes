# ROS 机器人技术 - 遇到的 ROS 问题总结

## 1 `/opt/ros/kinetic/share/pr2_motor_diagnostic_tool/plugin.xml` has no Root Element. This likely means the XML is malformed or missing..

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/rqt_tf_tree_error.png)

解决方法很容易：新建一个 `/opt/ros/kinetic/share/pr2_motor_diagnostic_tool/plugin.xml`，在里面加上根节点即可

```xml
<class_libraries>

</class_libraries>
```

