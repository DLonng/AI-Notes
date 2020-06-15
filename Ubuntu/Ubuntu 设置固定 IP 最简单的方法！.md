因为鼠标共享软件需要使用网络连接，台式机的 IP 总是变化导致笔记本每次重启都要重新配置 IP，所以我给 ubuntu 设置了固定 IP，方法很简单，直接再系统设置里面配置就可以了。

### 1. 查看默认网络配置

打开系统「Setting」，打开「Network」：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ubuntu_setting_network.png)

我连接的有线网络：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/wired_config.png)

启动终端，「ifconfig」查看下掩码：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/mask.png)

这里记录下你的配置，后续要用到：

- IPv4：ip 地址
- Default Route：网关地址
- Mask：255.255.255.0
- DNS：我这有 2 个DNS

### 2. 手动添加固定 IP

打开 Network 的 「Setting」，界面可能不太一样：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/wired_setting.png)

点击「IPv4」，Address 选项设置为「Manual」手动模式，其他配置如下：

- 设置一个静态的 IP，为了防止与其他同学的电脑冲突，我设置了 210 的地址，即：192.168.0.210，建议你也设置一个 200+ 的地址
- 其他的 Netmask、Gateway、DNS 跟之前一样

配置好后点击右下角的「Apply」，然后重启电脑就 ok 了！

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/config_static_ip.png)