台式机有 2 个硬盘，一个固态，一个机械，每次启动 Ubuntu 后要手动从文件管理器里面挂载，并且挂载的权限都是 777，所以按照网上的方法简单配置了下自动挂载，并设置普通的权限，这是挂载一个分区的结果，目录权限设置为 755，文件权限设置为 644：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/mount_data1.png)

下面简单介绍下过程，挺简单的，一行命令即可搞定！

### 配置 /etc/fstab 文件

打开 `/etc/fstab` 文件，在这个文件中配置硬盘自动挂载命令：

```shell
sudo gedit /etc/fstab
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/etc_fstab.png)

在末尾添加一行自己硬盘的挂载命令：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/add_new_mount_cmd.png)

```shell
UUID=723EC5D43EC59191      /home/dlonng/data1    ntfs    defaults,utf8,uid=1000,gid=1000,dmask=022,fmask=133     0       0
```

简单解释下：

- `UUID`：要挂载硬盘的 `UUID`，通过 `lsblk -f` 查看

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/lsblk_uuid.png)

- `/home/dlonng/data1`：要挂载到的目录，记得在挂载前**新建**这个目录！
- `ntfs`：挂载硬盘的文件类型，我的是 ntfs
- `default`：默认自动挂载
- `utf8`：挂载字符编码
- `uid`, `gid`：挂载这个硬盘的用户 id 和组 id，通过 `id username` 命令查看

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/uid_gid.png)

- `dmask`：挂载的目录权限掩码，目录权限 = 777 - dmask = 777 - 022 = 755 = rwx rx rx
- `fmask`：挂载的文件权限掩码，文件权限 = 777 - dmask = 777 - 133 = 644 = rw  r  r
- `0`：设置不自动备份
- `0`：设置开机不自检

按照你硬盘的信息，修改上面的命令，保存后重启即可看到硬盘自动挂载成功啦！

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ubuntu_mount_reboot.png)

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/yingliu_code/yinliu_code.png)