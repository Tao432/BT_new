# Usage
## 1. Required Libraries 
* vtk (A dependency library for PCL installation, need to check Qt during compilation)
* PCL

## 2. Prerequisites
It might be due to some incorrect settings in my publish/subscribe configurations. Using ROS2's default FastDDS causes significant lag during program execution. The reason hasn't been identified yet. Please follow the steps below to change the DDS to cyclonedds.

### 2.1 Install cyclonedds
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 2.2 Change default DDS
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 Verify the change
```
ros2 doctor --report | grep "RMW middleware"
```
If the output shows rmw_cyclonedds_cpp, the modification is successful.

## 3. Building the Code
In the workspace root directory:
### 3.1 Build all packages
```
colcon build
```

### 3.2 Build only the path planning part
If you only need to compile the path planning related packages, use the following command:
```
colcon build --packages-up-to ego_planner
```
This will build the ego_planner package and all its dependencies (including bspline_opt, path_searching, plan_env, traj_utils, etc.).

If you only want to build a single package, use:
```
colcon build --packages-select ego_planner
```

After building, remember to source the environment:
```
source install/setup.bash
```

## 4. Running the Code
### 4.1 Launch Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 3.2 Run the planning program
Open a new terminal and execute:
* Single drone
```
ros2 launch ego_planner single_run_in_sim.launch.py 
```
* swarm
```
ros2 launch ego_planner swarm.launch.py 
```
* large swarm
```
ros2 launch ego_planner swarm_large.launch.py  
```
* Additional parameters (optional):
    * use_mockamap:Map generation method. Default: False (uses Random Forest), True uses mockamap.
    * use_dynamic:Whether to consider dynamics. Default: False (disabled), True enables dynamics.
```
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```
# 使用方法
## 1. 需要的库 
* vtk(是安装PCL的依赖库，编译时需要勾选Qt)
* PCL

## 2. 前置条件
可能是我一些发布订阅的设置写的不太对，使用ROS2默认的FastDDS会导致程序运行很卡，目前还没找到原因，所以请按照下述方法将DDS修改为cyclonedds

### 2.1 安装cyclonedds
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### 2.2 修改默认的DDS
```
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 检查是否修改成功
```
ros2 doctor --report | grep "RMW middleware"
```
输出显示rmw_cyclonedds_cpp则说明修改成功

## 3. 编译代码
在工作空间根目录下：
### 3.1 编译所有包
```
colcon build
```

### 3.2 只编译路径规划部分
如果只需要编译路径规划相关的包，可以使用以下命令：
```
colcon build --packages-up-to ego_planner
```
这会编译 ego_planner 包及其所有依赖（包括 bspline_opt, path_searching, plan_env, traj_utils 等）。

如果只想编译单个包，可以使用：
```
colcon build --packages-select ego_planner
```

编译完成后记得source环境：
```
source install/setup.bash
```

## 4. 代码运行
### 4.1 运行Rviz
```
ros2 launch ego_planner rviz.launch.py 
```
### 4.2 运行规划程序
新开一个终端，输入以下指令
* 单机
```
ros2 launch ego_planner single_run_in_sim.launch.py 
```
* swarm
```
ros2 launch ego_planner swarm.launch.py 
```
* large swarm
```
ros2 launch ego_planner swarm_large.launch.py  
```
* 附加参数，可以选择地图生成模式以及是否考虑动力学
    * use_mockamap:地图生成方式，默认为False，False时使用Random Forest, True时使用mockamap
    * use_dynamic:是否考虑动力学，默认为False, False时不考虑, True时考虑
```
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```