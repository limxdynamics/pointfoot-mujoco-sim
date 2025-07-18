# English | [中文](README_cn.md)
# tron1-mujoco-sim Usage Guide

## 1. Run the Simulation

### Step 1: Open a terminal

### Step 2: Clone the MuJoCo simulation code

```bash
git clone --recurse https://github.com/limxdynamics/tron1-mujoco-sim.git
```

### Step 3: Install the motion control SDK

#### For Linux x86_64:

```bash
pip install tron1-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl
```

#### For Linux aarch64:

```bash
pip install tron1-mujoco-sim/limxsdk-lowlevel/python3/aarch64/limxsdk-*-py3-none-any.whl
```

### Step 4: Set Robot Type

Use the following shell command to list available robot types:

```bash
tree -L 1 tron1-mujoco-sim/robot-description/pointfoot
```

Example output:

```
tron1-mujoco-sim/robot-description/pointfoot
├── PF_P441A
├── PF_P441B
├── PF_P441C
├── PF_P441C2
├── PF_TRON1A
├── SF_TRON1A
└── WF_TRON1A
```

Take `PF_P441C` as an example (replace it with your actual robot type):

```bash
echo 'export ROBOT_TYPE=PF_P441C' >> ~/.bashrc && source ~/.bashrc
```

### Step 5: Run the MuJoCo simulator

```bash
python tron1-mujoco-sim/simulator.py
```

---

## 2. Compile and Run the Controller

### Step 1: Open a terminal

### Step 2: Install required build tools

```bash
sudo apt update
sudo apt install -y cmake build-essential
```

### Step 3: Compile the SDK example controller

```bash
cd tron1-mujoco-sim/limxsdk-lowlevel
mkdir -p build
cd build
cmake ..
make
```

### Step 4: Run the example controller

```bash
./examples/pf_groupJoints_move
```
