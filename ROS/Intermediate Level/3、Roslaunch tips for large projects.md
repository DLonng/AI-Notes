# Roslaunch tips for large projects

## 管理标签和环境变量
```
<include file="$(find pr2_alpha)/$(env ROBOT).machine" />
```
```
export ROBOT=pre
```
```
$(env ROBOT).machine = pre.machine
```

用到再来看。