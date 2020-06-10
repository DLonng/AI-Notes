# Autonomous Navigation for BigDog

## 系统架构

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/bigdog-robot.png)

我们的一般技术方法利用来自两个环境传感器的数据来识别障碍物，计算穿过障碍物或围绕障碍物的轨迹，并命令步态控制系统跟踪该轨迹。

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/bigdog-soft-arch.png)

### Point-cloud Segmentation and Object Tracking

1. 第一步是将 LIDAR 点云和基于立体的地形图提供的障碍点分割成不同的对象，通过合并相距不到 0.5 米的单个点，将稀疏的 3D 点云分割为单个对象。
2. 随时间推移跟踪由分割算法产生的对象，我们采用具有启发式约束的贪婪迭代算法来完成此任务，通过将点云分割为对象并随时间进行跟踪，该机器人可以在地面坡度变化适中且障碍物种类繁多的环境中适当运行，这些障碍物包括树木，巨石，倒下的原木，墙壁。
3. 树木和墙壁主要由 LIDAR 识别，巨石和倒下的原木由立体视觉系统识别

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/bigdog-perception.png)

主要使用：图像、点云 目标检测与跟踪