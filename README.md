# SLAM FPGA Competition Project Instructions

This repository is the official baseline project for the SLAM FPGA Algorithm Competition, providing a complete point cloud localization demo framework. Please read this document carefully before environment setup and algorithm development.

---

## Contest & Communication

- **Official Contest Page**: Please check the official contest website for detailed rules and procedures.
- **Q&A and Support**: Join the official Slack workspace ([invite link: xxxxxx]) for questions and technical communication.

---

## Development Board & Basic Environment

### 1. Kria KV260 Board OS Setup

- Follow the [Xilinx official getting started guide](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit/kv260-getting-started/getting-started.html) to install Ubuntu 22.04 on your Kria KV260 board.
- **Boot Firmware Note**: If you have boot firmware (Boot FW) compatibility issues at first boot:
    - First boot with Ubuntu 20.04, install the Xilinx software tools.
    - Use the `xlnx-config --xmutil <cmd>` utility to update the firmware, following the [official instructions](https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/1641152513/Kria+K26+SOM#Boot-FW-update-with-xmutil).
    - Please follow all instructions strictly as described on the official page.

### 2. Installing the PYNQ System (Optional)

- Follow the steps in the [Kria-PYNQ official repository](https://github.com/Xilinx/Kria-PYNQ) to install the PYNQ system and Jupyter Notebook environment.
- It is recommended to get familiar with accessing and operating the board via Jupyter.

---

## SLAM Baseline Project Usage

### 1. Clone the Project & Directory Overview

Clone the repository on your host or the development board:

git clone <your_project_git_url>
cd slam_fpga_competition
Directory Structure:

slam_fpga_competition/
├── src/                 # Main program and algorithm directory
│   ├── main.cpp         # Main entry point
│   └── slam_algorithm/  # Algorithm implementations (extendable/replaceable)
├── data/                # Dataset directory (user-prepared)
│   └── kitti/
├── scripts/             # Helper scripts (build, run, evaluation)
├── output/              # Output results and logs
├── doc/                 # Documentation
├── README.md            # Quick start guide
├── env_setup.md         # Environment dependency instructions
├── requirements.txt     # Python dependencies

### 2. Dataset Preparation
Download and organize the KITTI dataset into data/kitti/00/velodyne/ (bin files).
Place the ground truth trajectory as data/kitti/00/groundtruth.txt.
See README.md for further dataset organization examples.

### 3. Environment Setup
#### 3.1 C++ and PCL Environment
sudo apt update
sudo apt install -y cmake libpcl-dev libeigen3-dev
#### 3.2 Python Dependencies
sudo apt install -y python3 python3-pip
pip3 install -r requirements.txt
####3.3 Build the Project
bash
mkdir -p build
cd build
cmake ../src
make -j4
cd ..
### 4. Running & Evaluation
#### 4.1 One-Click Run (Recommended)
bash
bash scripts/run_host.sh
This script will automatically build (if not already built), run SLAM localization, and output the trajectory to output/results/.
It will also automatically evaluate localization accuracy and save results in output/logs/eval.log.
#### 4.2 Manual Run
bash
./build/main data/kitti/00/velodyne data/kitti/00/groundtruth.txt
python3 scripts/evaluate.py output/results/traj_est.txt output/results/traj_gt.txt
### 5. Output Files
output/results/traj_est.txt: Estimated trajectory from your algorithm
output/results/traj_gt.txt: Ground truth trajectory
output/logs/: Logs and evaluation results for each step
### 6. Algorithm Extension
Teams can implement or replace their own SLAM algorithm in the src/slam_algorithm/ directory. Please refer to the interface in icp_localization.hpp.
As long as the interface remains consistent, your implementation can be seamlessly tested and evaluated by the framework.
Reference & Technical Support
Please refer to the official contest page for detailed rules, dataset download links, and evaluation standards.
For technical discussions and support, join the official Slack workspace (link provided above).
If you encounter environment setup or code running issues, please first consult env_setup.md and README.md. For further questions, please ask in the Slack group.

基本运行流程。
# 1. 安装依赖环境
## 1.1 安装C++依赖
更新软件源
sudo apt update
安装CMake
sudo apt install -y cmake
安装PCL
sudo apt install -y libpcl-dev
安装Eigen
sudo apt install -y libeigen3-dev
检查gcc/g++版本
g++ --version
推荐g++ 7及以上

## 1.2 安装Python3及numpy
安装Python3
sudo apt install -y python3 python3-pip
安装numpy
pip3 install numpy


# 2. 工程目录准备
## 假设在你的主目录
cd slam_fpga_competition

# 3. 数据准备
## 3.1 下载KITTI数据
进入数据目录
cd data/kitti/
假设你下载的是 00 序列，解压后目录为 data/kitti/00/velodyne/*.bin
groundtruth.txt 需要从KITTI官方的 poses/00.txt 拷贝过来并命名为 groundtruth.txt
注意：
velodyne/ 目录下必须是连续编号（000000.bin、000001.bin……）。
groundtruth.txt为每帧一个3x4矩阵一行。

# 4. 工程编译
## 4.1 创建构建目录并编译
cd slam_fpga_competition   # 回到项目根目录
mkdir -p build
cd build
cmake ../src
make -j4
# 编译成功后，会在build/目录下生成 main 可执行文件
# 5. 运行DEMO定位与评测
## 5.1 一键脚本运行（推荐）
cd ..
bash scripts/run_host.sh
这个脚本会自动编译（如果没编译过）
调用 ./build/main data/kitti/00/velodyne data/kitti/00/groundtruth.txt
输出轨迹及结果到 output/results/
