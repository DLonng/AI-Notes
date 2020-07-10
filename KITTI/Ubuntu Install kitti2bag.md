项目需要测试建图效果，但是目前我们自己的语义分割训练的效果不是太好，外参矩阵标定的精度也不高，所以打算使用 KITTI 数据集作为测试，为此需要将 KITTI 数据集转为 bag 包方便回放，这里要安装一个 [kitti2bag](https://github.com/tomas789/kitti2bag) 的工具，记录下安装过程，并不是一步到位。

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

但是我已经安装成功了，所以我查找了下 kitti2bag 的位置，我先使用卸载命令查看 kitti2bag 提示的安装位置在哪里：

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

重启终端，再执行 `kitti2bag`，打印出使用信息说明可以使用：

```shell
kitti2bag

usage: kitti2bag [-h] [-t DATE] [-r DRIVE]
                 [-s {00,01,02,03,04,05,06,07,08,09,10,11,12,13,14,15,16,17,18,19,20,21}]
                 {raw_synced,odom_color,odom_gray} [dir]
kitti2bag: error: too few arguments
```

配一张我执行这个过程的图片：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/kitti2bag_cp.png)

### 3. 使用 kitti2bag 转换 bag

下载了 KITTI 包后，我把数据集和标定文件都解压到当前目录，注意它们都会解压到以**时间命名**的文件夹中：

```
unzip KITTI_2011_09_26_drive_0020_sync
unzip 2011_09_26_calib.zip
```

上面两者都解压到 `2011_09_26` 目录下，然后在解压后的目录的上一级运行如下的转换命令，`raw_synced` 表示下载的数据集是否是同步过的，我下载是是同步的，所以加上了这句，不过基本都是用同步的：

```shell
kitti2bag -t 2011_09_26 -r 0020 raw_synced .
```

等待转换完成，即可看到生成的 bag 包，打印下 info 看下信息：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/kitti2bag_result_info.png)

大功告成！

参考链接：

- https://github.com/tomas789/kitti2bag/issues/20