## 一、整理文件

- 清空回收站
- 删除下载文件夹不需要的文件
- 删除其他不需要的文件

## 二、关闭非系统服务

- 关闭 samba

  - ```shell
    sudo /etc/init.d/samba stop
    ```

- 关闭 synergy

## 三、备份过程



## 四、恢复

分区：

- swap: 8G
- boot/efi: 200
- /: 30
- /home:60