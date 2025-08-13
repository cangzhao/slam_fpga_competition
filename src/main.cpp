//  顶层主程序，负责数据读取、调用SLAM流程、调用FPGA加速模块。

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slam_algorithm/icp_localization.hpp"

using namespace std;

vector<string> get_scan_files(const string& scan_dir) {
    vector<string> files;
    for (int i=0;; ++i) {
        char buf[256];
        sprintf(buf, "%s/%010d.bin", scan_dir.c_str(), i);
        ifstream ifs(buf);
        if (!ifs) break;
        files.push_back(string(buf));
    }
    return files;
}

vector<Eigen::Matrix4d> readTraj(const string& gt_path) {
    vector<Eigen::Matrix4d> traj;
    ifstream ifs(gt_path);
    string line;
    while(getline(ifs, line)) {
        istringstream ss(line);
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        for(int i=0;i<3;i++)
            for(int j=0;j<4;j++)
                ss >> pose(i,j);
        traj.push_back(pose);
    }
    return traj;
}

void writeTraj(const string& out_path, const vector<Eigen::Matrix4d>& poses) {
    ofstream ofs(out_path);
    for (const auto& pose : poses) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j)
                ofs << pose(i, j) << ((i == 2 && j == 3) ? "" : " ");
        ofs << endl;
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: ./main <velodyne_dir> <groundtruth.txt>" << endl;
        return -1;
    }
    string scan_dir = argv[1];
    string gt_path = argv[2];

    // 自动创建output/results目录
    system("mkdir -p output/results");
    system("mkdir -p output/logs");

    auto scan_files = get_scan_files(scan_dir);
    cout << "Found " << scan_files.size() << " scans." << endl;

    // 计时定位流程
    auto t1 = chrono::steady_clock::now();
    auto poses = slam_algorithm::icp_localization(scan_files);
    auto t2 = chrono::steady_clock::now();
    double duration = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

    // 写入轨迹
    writeTraj("output/results/traj_est.txt", poses);

    // 拷贝GT轨迹
    ifstream src(gt_path); ofstream dst("output/results/traj_gt.txt");
    dst << src.rdbuf();

    // 写入日志
    ofstream log("output/logs/run.log");
    log << "Time: " << duration << " s\n";
    log << "Frames: " << poses.size() << "\n";
    log << "FPS: " << poses.size() / duration << endl;
    log.close();

    cout << "Done. Time: " << duration << " s, FPS: " << poses.size() / duration << endl;
    return 0;
}
