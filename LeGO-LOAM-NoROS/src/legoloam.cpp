#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include "imageProjection.h"


int main(int argc, char const *argv[])
{
    lego_loam::ImageProjection IP;
    pcl::io::loadPCDFile("/mnt/d/Download/Dataset/Kitti/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/pcd/0000000001.pcd", *(IP.laserCloudIn));

    IP.Process();
    
    std::string filename = "/mnt/d/Download/Dataset/test.pcd";
    if (pcl::io::savePCDFile(filename, *(IP.laserCloudIn)) == -1)
    {
        PCL_ERROR("Failed to save PCD file\n");
        return -1;
    }
    std::cout << "Saved point cloud to " << filename << std::endl;

    return 0;
}
