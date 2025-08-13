# 在KV260 ARM端执行主SLAM流程的脚本。
#!/bin/bash
set -e

# 获取当前脚本的目录，再切回根目录
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$PROJECT_ROOT"

mkdir -p output/results
mkdir -p output/logs

mkdir -p build
cd build
if [ ! -f Makefile ]; then
    cmake ../src
fi
make
cd ..

./build/main data/kitti/00/velodyne data/kitti/00/groundtruth.txt

evo_ape kitti output/results/traj_gt.txt output/results/traj_est.txt -a | tee output/logs/eval.log
echo "" >> output/logs/eval.log
evo_rpe kitti output/results/traj_gt.txt output/results/traj_est.txt -a | tee -a output/logs/eval.log
evo_traj kitti /home/cangzhao/slam_fpga_competition/output/results/traj_est.txt --ref /home/cangzhao/slam_fpga_competition/output/results/traj_gt.txt -p -a