#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptimization.h"

Eigen::Isometry3d updatePose(const Odometry& odometry, const Eigen::Isometry3d& previousPose)
{
    Eigen::Quaterniond quat(odometry.orientationW, odometry.orientationX, odometry.orientationY, odometry.orientationZ);
    Eigen::Isometry3d currentPose = Eigen::Isometry3d::Identity();
    currentPose.rotate(quat);
    currentPose.translate(Eigen::Vector3d(odometry.positionX, odometry.positionY, odometry.positionZ));

    return previousPose * currentPose;
}

void saveToFile(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){
    std::string filename = "/mnt/d/Download/Dataset/test2.pcd";
    // std::cout << FA.laserCloudSurfLastMetadata.timestamp << std::endl;
    if (pcl::io::savePCDFile(filename, *(cloud)) == -1)
    {
        PCL_ERROR("Failed to save PCD file\n");
        return;
    }
    std::cout << "Saved point cloud to " << filename << std::endl;
}


int main(int argc, char const *argv[])
{   
    std::string base_path = "/mnt/d/Download/Dataset/Kitti/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/pcd/";
    std::string extension = ".pcd";

    lego_loam::ImageProjection IP;
    lego_loam::FeatureAssociation FA;
    lego_loam::MapOptimization MO;

    int time = 10;
    pcl::PointCloud<pcl::PointXYZ>::Ptr poses(new pcl::PointCloud<pcl::PointXYZ>);

    // boost::filesystem::path directory_path("/mnt/d/Download/Dataset/Test");
    boost::filesystem::path directory_path("/mnt/d/Download/Dataset/Kitti/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/pcd");
    if(boost::filesystem::exists(directory_path) && boost::filesystem::is_directory(directory_path)) {
        for (const auto& entry : boost::filesystem::directory_iterator(directory_path)) {

            // Do something with the filename
            boost::filesystem::path p = entry.path();
            std::string filename = p.filename().string();
            filename = "/mnt/d/Download/Dataset/Kitti/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/pcd/" + filename;
            std::cout << filename << std::endl;

            pcl::io::loadPCDFile(filename, *(IP.laserCloudIn));
            IP.laserCloudInMetadata.timestamp = time;

            IP.Process();

            FA.adjustInput(IP.segMsg, IP.segmentedCloudMetadata, IP.outlierCloudMetadata);
            FA.runFeatureAssociation();

            MO.adjustLaserInput(FA.outlierCloudMetadata, FA.laserCloudCornerLastMetadata, FA.laserCloudSurfLastMetadata, FA.laserOdometry);
            MO.run();

            // pcl::PointXYZ point;
            // point.x = static_cast<float>();
            // point.y = static_cast<float>(FA.laserOdometry.positionY);
            // point.z = static_cast<float>(FA.laserOdometry.positionZ);
            // poses->push_back(point);
            if (MO.cloudKeyPoses3D->points.size() > 0){
                saveToFile(MO.cloudKeyPoses3D);
            }
            time++;

            IP.resetParameters();
        }
    }

    // saveToFile(poses);

    /*
    for (int i = 0; i <= 50; ++i) {
        std::stringstream ss;
        ss << std::setw(10) << std::setfill('0') << i;
        std::string filename = base_path + ss.str() + extension;
        std::cout << filename << std::endl;

        // Do something with the filename
        pcl::io::loadPCDFile(filename, *(IP.laserCloudIn));
        IP.laserCloudInMetadata.timestamp = time;

        IP.Process();

        FA.adjustInput(IP.segMsg, IP.segmentedCloudMetadata, IP.outlierCloudMetadata);
        FA.runFeatureAssociation();
        time++;

        // pose = updatePose(FA.laserOdometry, pose);
        pcl::PointXYZ point;
        point.x = static_cast<float>(FA.laserOdometry.positionX);
        point.y = static_cast<float>(FA.laserOdometry.positionY);
        point.z = static_cast<float>(FA.laserOdometry.positionZ);
        poses->push_back(point);

        IP.resetParameters();
    } */
    

    return 0;
}
