新买的电脑装的的英伟达独立显卡，虽然 Ubuntu 上有开源 Nvida 驱动可以用，但是界面总是卡卡的，所以安装了下官方的驱动，瞬间丝滑无比，以下是我的方法，简单粗暴。

### Ubuntu 16.04 安装 Nvida 显卡

推荐命令行安装，简单有效，先添加源：

```shell
sudo add-apt-repository ppa:graphics-drivers/ppa

sudo apt-get update
```

然后查看当前机器显卡推荐安装的驱动：

```shell
ubuntu-drivers devices

== /sys/devices/pci0000:00/0000:00:01.0/0000:01:00.0 ==
modalias : pci:v000010DEd00001D01sv000010DEsd00001D01bc03sc00i00
vendor   : NVIDIA Corporation
driver   : nvidia-415 - third-party free
driver   : nvidia-430 - third-party free recommended
driver   : xserver-xorg-video-nouveau - distro free builtin
driver   : nvidia-384 - distro non-free
driver   : nvidia-418 - third-party free
driver   : nvidia-410 - third-party free

== cpu-microcode.py ==
driver   : intel-microcode - distro free
```

推荐安装（recommended） nvidia-430，直接自动安装推荐的版本：

```shell
sudo ubuntu-drivers autoinstall
```

但是我安装过程中提示缺少一个依赖库，所以为了解决依赖，我使用 `aptitude `单独安装 nvidia-430：

```shell
sudo aptitude install nvidia-430
```

没有安装 aptitude 直接 apt 安装即可：

```shell
sudo apt install aptitude
```

使用 aptitude 的时候，会让你选择是否更改当前的依赖项的变化，默认「Y」是保留当前系统的状态，不解决依赖，选择「n」则进行依赖库的安装与卸载等，建议在安装的时候查看控制台输出，看看是否有误删除的依赖库，确认没有问题后，再选「n」进行下一步即可，你也可以到网上搜索下 aptitude 的用法。

安装完重启电脑即可，查看下当前显卡的信息：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/nvida-smi.png)

有类似输出说明安装成功！话说我应该买个更好的显卡 =_=