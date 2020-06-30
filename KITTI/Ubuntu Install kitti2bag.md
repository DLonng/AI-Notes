项目需要测试，将 KITTI 转为 bag 包方便回放，这里要安装一个 [kitti2bag](https://github.com/tomas789/kitti2bag) 的工具，记录下安装过程，并不是一步到位。。。

### 1. 更新 pip 安装 kitti2bag

安装前提示我更新 pip，我用的 Python2.7 使用的是 pip：

```shell
pip install --upgrade pip
```

安装 kitti2bag：

```shell
pip install kitti2bag
```

### 2. 运行 kitti2bag 提示 command no found

安装完后直接命令行运行 `kitti2bag` 提示命令找不到：

```shell
kitti2bag

command no found!
```

但是我已经安装成功了，所以我查找了下 kitti2bag 的位置，我先卸载 kitti2bag 查看提示的安装位置在哪里：

```shell
pip uninstall kitti2bag
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/kitti2bag_install_path.png)

发现在 `/home/dlonng/.local/bin/kitti2bag` 下，所以我直接把 kitti2bag 拷贝到 `/usr/bin/` 下：

```shell
sudo cp /home/dlonng/.local/bin/kitti2bag /usr/bin/

ls /usr/bin/kitti2bag

kitti2bag
```

重启终端，再执行 `kitti2bag`，打印使用信息即可：

```shell
kitti2bag

usage: kitti2bag [-h] [-t DATE] [-r DRIVE]
                 [-s {00,01,02,03,04,05,06,07,08,09,10,11,12,13,14,15,16,17,18,19,20,21}]
                 {raw_synced,odom_color,odom_gray} [dir]
kitti2bag: error: too few arguments
```

配一张我执行这个过程的图片：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/kitti2bag_cp.png)

参考链接：

- https://github.com/tomas789/kitti2bag/issues/20
- https://zhuanlan.zhihu.com/p/76315087