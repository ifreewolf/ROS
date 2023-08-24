# 第5章 ROS常用组件

在ROS中内置一些比较实用的工具，通过这些工具可以方便快捷的实现某个功能或调试程序，从而提高开发效率，本章主要介绍ROS中内置的如下组件：

- TF坐标变换，实现不同类型的坐标系之间的转换；
- rosbag用于录制ROS节点的执行过程并可以重放该过程；
- rqt工具箱，集成了多款图形化的调试工具。

本章预期达成的学习目标：

- 了解TF坐标变换的概念以及应用场景；
- 能够独立完成TF案例：小乌龟跟随；
- 可以使用rosbag命令或编码的形式实现录制与回放；
- 能够熟练使用rqt的图形化工具。

<B>案例演示：</B>小乌龟跟随实现，该案例是ros中内置案例，终端下键入启动命令

```bash
roslaunch turtle_tf2 turtle_tf2_demo_cpp.launch 或
roslaunch turtle_tf2 turtle_tf2_demo.launch
```

键盘可以控制一只乌龟运动，另一只跟随运动。

<div align="center">
    <img src="./image/TF坐标变换.gif" />
</div>