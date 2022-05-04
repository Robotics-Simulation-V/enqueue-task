#  使用方法

---

## 准备工作

* 首先先下载turtlebot3

进入到自己工作空间的src目录下，然后下载源代码，进行编译：

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws/
$ catkin_make
```



* 把指定文件替换

**`xxx`为自己的用户名**

1、把`final.world` 替换掉`/home/xxx/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds`下同名文件

2、把`turtlebot3_world.launch`替换掉`/home/xxx/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch`下同名文件

3、把`mapfinal222.yaml` 和`mapfinal222.pgm` 放入`/home/xxx/catkin_ws/src/turtlebot3/turtlebot3_navigation/maps`

把`turtlebot3_navigation.launch`替换掉

`/home/xxx/catkin_ws/src/turtlebot3/turtlebot3_navigation/launch`下同名文件

4、将`amcl.launch`替换掉`/home/xxx/catkin_ws/src/turtlebot/amcl.launch下同名文件`

5、进入工作空间下执行`catkin_make`



---

## 任务一

* 简述

**撞击指定颜色的小球**

* 运行 

`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

`roslaunch task1 get_to_the_ball.launch`

---

## 任务二

* 简述

**走出迷宫**

* 运行

`roslaunch task2 to_designated_goal.launch`

---

## 任务三

* 简述

**巡线**

* 准备巡线

1 、将 

`A_ground_plane`放在 `/home/xxx/.gazebo/models`目录下    

找不到.gazebo的话按下 <kbd>Ctrl</kbd>+<kbd>h</kbd> 显示隐藏的文件

* 运行

在gazebo中点击insert找到A_ground_plane贴在地上

将小车放在线上

`roslaunch task3 follow_the_line.launch`

---

## 任务四

* 简述

**两辆小车编队运行**

* 准备

1、把`task4`下四个文件夹放在你`工作空间/src`下

2、打开`/src/map_launcher/map/tas4_map.yaml`将`/home/magci/tas4_map.pgm`的`magci`改为自己的用户名并把`tas4_map.pgm`放在主目录下

3、进入工作空间下执行`catkin_make`

* 运行

`roslaunch multi_turtlebots_sim main.launch`

`roslaunch task4 task4.launch ` 即可让小车1追着小车2

使用`rviz`上的`2D Nav Goal`让小车2动起来

