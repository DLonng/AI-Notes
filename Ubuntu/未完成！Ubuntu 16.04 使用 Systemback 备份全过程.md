为了把所有的工作环境都迁移到新的电脑上，我把原来台式机上的系统进行备份，然后直接安装到新的电脑上，立马可以干活，以下是我的备份过程，基本适用与所有的 Ubuntu 版本。

## 一、删除不必要的文件

如果系统中存在不需要或者非常大的文件会导致备份过程非常慢，建议先把以下 2 者处理：

- 清空回收站
- 备份或删除下载目录的文件

然后用 `Disk Usage Analyzer` 命令查看下系统当前文件夹的用量，看看是否有非常大的文件：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/disk_usage_analyzer.png)

比如我第一次备份时没发现 Matlab 竟然占用了 17G 的空间，而我目前又不经常用，所以第二次备份就直接把 Matlab 卸载了，之后备份进程快多了。

## 二、关闭非系统服务

我平常跟笔记本用 `synergy + smaba` 通信，所以先把它们关掉：

- 关闭` samba` 服务器

  ```
  sudo /etc/init.d/samba stop
  ```

- 退出鼠标键盘共享软件 `synergy`

## 三、备份过程

我用的是 `Systemback` 软件，安装方式如下：

```shell
# 先更新下系统保证没问题
sudo apt update

# 添加 systemback 源到系统
sudo add-apt-repository ppa:nemh/systemback

# 更新源
sudo apt update

# 安装
sudo apt install systemback
```

在第一步把不需要的大文件都删除后，就可以直接启动 Systemback 备份，好像启动时要输入 root 密码：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/systemback_root.png)

输入后 OK 即可进入主界面：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/live_system_create.png)

点击 `Live system create` 进入备份界面：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/start_create_new.png)

注意：

- 检查系统的 `/home` 是否有足够空间存放备份的镜像
- 勾选包含用户数据文件，因为用户数据可能包含一些软件的配置文件等，很重要
- 如果一切准备就绪点击 「Create new」即可开始备份

在备份的过程中，如果忘记什么操作随时都可以中断，不会出问题，备份完后右上角会显示备份后的 sblive 文件，然后我们插入一个 U 盘，把备份的镜像直接写入 U 盘中，注意把 U 盘内的资料备份，因为写入镜像前会把 U 盘格式化：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/write_to_target.png)

我之前备份的文件我删除了，当你备份完后「Write to target」会变的可以点击，点击后开始把镜像写入 U 盘中，写完后把 U 盘安全弹出后，即可进入下一步来恢复系统。

## 四、恢复系统镜像

我的电脑默认装了 Win10，所以可以直接装 Ubuntu，因为 Ubuntu 的 GRUB 可以引导 Win10，所以安装完后 2 个系统都可以正常启动。如果你的电脑准备装双系统，建议先装 Win 再装 Ubuntu，这样会省很多事情。

### 4.1 开机从 U 盘启动

先插上 U 盘，然后到网上搜索下不同主板选择开启启动项的按钮，我的主板是技嘉按 F12，如果是联想好像也是 F12，选择 U 盘启动：

手机拍照

然后选择第一项 `Boot Live system` 进入 U 盘内的 Ubuntu 系统：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/boot_live_system.png)

进入系统后再次启动 `System back`，进入系统安装：

分区：

- swap: 8G，物理内存的整数倍
- boot/efi: 1 G
- /: 尽量大，
- /home:尽量大，

根目录：home = 1:1 或者 1:2





#### Intel NUC 256G 分区

- boot/efi: 1G = 1024 M
- swap: 16G = 16384M
- /：100 G = 102400 M
- /home: 139G = remain





参考博客：

- https://blog.csdn.net/u012052268/article/details/77145427