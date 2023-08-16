#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include "imageProjection.h"
#include "featureAssociation.h"


int main(int argc, char const *argv[])
{
    lego_loam::ImageProjection IP;
    pcl::io::loadPCDFile("/mnt/d/Download/Dataset/Kitti/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/pcd/0000000001.pcd", *(IP.laserCloudIn));
    IP.laserCloudInMetadata.timestamp = 123456.78901;
    // std::cout << IP.laserCloudInMetadata.cloud->points.size() << std::endl;

    IP.Process();

    // std::cout << std::fixed << std::setprecision(3) << IP.segmentedCloudMetadata.timestamp << std::endl; // prints "12345.678"
    // std::cout << IP.segmentedCloudMetadata.frame_id << std::endl;

    lego_loam::FeatureAssociation FA;
    FA.adjustInput(IP.segMsg, IP.segmentedCloudMetadata, IP.outlierCloudMetadata);
    FA.runFeatureAssociation();

    
    std::string filename = "/mnt/d/Download/Dataset/test.pcd";
    if (pcl::io::savePCDFile(filename, *(FA.laserCloudCornerLast)) == -1)
    {
        PCL_ERROR("Failed to save PCD file\n");
        return -1;
    }
    std::cout << "Saved point cloud to " << filename << std::endl;
    

    return 0;
}
