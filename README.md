# pointfoot-mujoco-sim



## 1. 搭建开发环境

我们建议在 Ubuntu 20.04 及以上版本上搭建基于 MuJoCo 的算法仿真开发环境。请在 Bash 终端中运行以下命令，以安装所需的依赖库：

```
sudo apt update
sudo apt install -y libglfw3 libglfw3-dev cmake build-essential
```



## 2. 编译运行仿真

- 打开一个 Bash 终端。

- 下载 MuJoCo 仿真器代码：

  ```
  git clone -b feature/mujoco_cpp --recurse https://github.com/limxdynamics/pointfoot-mujoco-sim.git
  ```

- 设置机器人类型

  - 通过 Shell 命令 `tree -L 1 pointfoot-mujoco-sim/robot-description/pointfoot` 列出可用的机器人类型：

    ```
    limx@limx:~$ tree -L 1 pointfoot-mujoco-sim/robot-description/pointfoot
    pointfoot-mujoco-sim/robot-description/pointfoot
    ├── PF_P441A
    ├── PF_P441B
    ├── PF_P441C
    └── PF_P441C2
    
    ```

  - 以`PF_P441C`（请根据实际机器人类型进行替换）为例，设置机器人型号类型：

    ```
    echo 'export ROBOT_TYPE=PF_P441A' >> ~/.bashrc && source ~/.bashrc
    ```

- 编译 MuJoCo 仿真器：

  ```
  cd pointfoot-mujoco-sim
  mkdir -p build
  cd build
  cmake ..
  make
  ```

- 运行 MuJoCo 仿真器：

  ```
  cd build
  ./simulator
  ```

  

## 3. 编译运行控制

- 打开一个 Bash 终端。

- 编译控制器SDK示例：

  ```
  cd pointfoot-sdk-lowlevel
  mkdir -p build
  cd build
  cmake ..
  make
  ```

- 运行控制器SDK示例：

  ```
  cd build
  ./examples/pf_groupJoints_move
  ```



## 4. 仿真展示

![](doc/simulator.gif)

