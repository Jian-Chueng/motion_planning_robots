# motion_planning_robots

### 生成workspace

```shell
mkdir -p ~/文件名/src

cd ~/文件名/src` 

catkin_init_workspace #  初始化workspace 生成CMakeLists.txt

cd ~/文件名/或者 cd .. # 回到上级目录

catkin_make # 生成build和devel文件夹

source devel/setup.bash # 将workspace路径添加到bash中
```

### commend line 

```shell
roscore      //第一个commend window

source devel/setup.bash      //第二个commend window

roslaunch grid_path_searcher demo.launch  //前提：已经把config加到demo.launch文件中

```

或者

```shell
roscore      //第一个commend window

rviz     //第二个commend window
	open config – src/grid_path_searcher/launch/rviz_confi/demo.rviz
	
source devel/setup.bash      //第三个commend window

 roslaunch grid_path_searcher demo.launch
```

