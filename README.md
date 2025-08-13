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
