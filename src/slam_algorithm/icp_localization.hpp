#pragma once
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sstream>

namespace slam_algorithm {

// 读取KITTI格式bin点云
inline pcl::PointCloud<pcl::PointXYZ>::Ptr readKITTIBin(const std::string &file_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::ifstream ifs(file_path, std::ios::binary);
    if (!ifs) {
        std::cerr << "Cannot open " << file_path << std::endl;
        return cloud;
    }
    while (!ifs.eof()) {
        float x, y, z, intensity;
        ifs.read((char *)&x, sizeof(float));
        ifs.read((char *)&y, sizeof(float));
        ifs.read((char *)&z, sizeof(float));
        ifs.read((char *)&intensity, sizeof(float));
        if (ifs.gcount() == 0) break;
        cloud->points.emplace_back(x, y, z);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

// ICP核心流程
inline std::vector<Eigen::Matrix4d> icp_localization(const std::vector<std::string>& scan_files) {
    std::vector<Eigen::Matrix4d> poses;
    Eigen::Matrix4d cur_pose = Eigen::Matrix4d::Identity();
    poses.push_back(cur_pose);

    for (size_t i = 1; i < scan_files.size(); ++i) {
        auto src = readKITTIBin(scan_files[i-1]);
        auto tgt = readKITTIBin(scan_files[i]);

        // 下采样加速
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(0.5, 0.5, 0.5);
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_ds(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_ds(new pcl::PointCloud<pcl::PointXYZ>());
        voxel.setInputCloud(src); voxel.filter(*src_ds);
        voxel.setInputCloud(tgt); voxel.filter(*tgt_ds);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(src_ds);
        icp.setInputTarget(tgt_ds);
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);

        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();
        cur_pose = cur_pose * T;
        poses.push_back(cur_pose);
        std::cout << "Frame " << i << " done, score=" << icp.getFitnessScore() << std::endl;
    }
    return poses;
}

} // namespace slam_algorithm
