/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#include <Parameters.h>
#include <SLAMBenchAPI.h>
#include <io/SLAMFrame.h>
#include <io/sensor/LidarSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <chrono>
#include <Eigen/Eigen>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>

#include "include/legoloam.h"
#include "utility"


const std::string default_yaml_path = "/deps/legoloam/configs/default.yaml";
const std::string dirname = "/mnt/d/Download/Dataset/Kitti/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/pcd/";

// Parameters
std::string lidar_name;
std::string yaml_path;

// Sensors
slambench::io::LidarSensor *lidar_sensor;

size_t frame_id = 0;

slambench::TimeStamp last_frame_timestamp;
double current_timestamp;

// Outputs
slambench::outputs::Output *pose_output;
slambench::outputs::Output *pointcloud_output;

// System
static lego_loam::LeGOLOAM legoloam;
// contains rotation only
Eigen::Matrix4f velo_2_lgrey = (Eigen::Matrix4f() << 9.999728e-01f,  7.027479e-03f, -2.255075e-03f,  0.000000e+00f,
                                                    -7.027555e-03f,  9.999753e-01f, -2.599616e-05f,  0.000000e+00f,
                                                     2.254837e-03f,  4.184312e-05f,  9.999975e-01f,  0.000000e+00f,
                                                     0.000000e+00f,  0.000000e+00f,  0.000000e+00f,  1.000000e+00f).finished();

Eigen::Matrix4f align_mat = (Eigen::Matrix4f() << -1.0,  0.0, 0.0, 0.0,
                                                   0.0, -1.0, 0.0, 0.0,
                                                   0.0,  0.0, 1.0, 0.0,
                                                   0.0,  0.0, 0.0, 1.0).finished();
Eigen::Matrix4f pose;


bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {

    slam_settings->addParameter(TypedParameter<std::string>("", "configuration", "path to configuration YAML file", &yaml_path, &default_yaml_path));

    return true;
}


bool sb_init_slam_system(SLAMBenchLibraryHelper *slam_settings) {
    
    // Declare Outputs
    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);

    pointcloud_output = new slambench::outputs::Output("PointCloud", slambench::values::VT_POINTCLOUD, true);
    pointcloud_output->SetKeepOnlyMostRecent(true);

    slam_settings->GetOutputManager().RegisterOutput(pose_output);
    slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);

    // Inspect sensors
    lidar_sensor = (slambench::io::LidarSensor*)slam_settings->get_sensors().GetSensor(slambench::io::LidarSensor::kLidarType);
    if (lidar_sensor == nullptr) {
        std::cerr << "Invalid sensors found, Lidar not found." << std::endl;
        return false;
    }

    // Start LeGO-LOAM

    if (!legoloam.Init()) {
        std::cerr << "Failed to initialize slam system." << std::endl;
        return false;
    }
    std::cout << "LeGO-LOAM initialized" << std::endl;

    return true;
}


bool sb_update_frame(SLAMBenchLibraryHelper *slam_settings , slambench::io::SLAMFrame *s) {
    
	if (s->FrameSensor == lidar_sensor) {

        last_frame_timestamp = s->Timestamp;
        current_timestamp = static_cast<double>(s->Timestamp.S) + static_cast<double>(s->Timestamp.Ns) / 1e9;
        legoloam.IP_->laserCloudInMetadata.timestamp = current_timestamp;
        
        float *fdata = static_cast<float*>(s->GetData());
        int count = s->GetSize()/(4 * sizeof(float));

        for(int i = 0; i < count; ++i) {
            float x = fdata[i*4];
            float y = fdata[i*4+1];
            float z = fdata[i*4+2];
            float intensity = 0.0;
            pcl::PointXYZI point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = intensity;
            legoloam.IP_->laserCloudIn->points.push_back(point);
        }
        // legoloam.IP_->laserCloudIn->width = legoloam.IP_->laserCloudIn->points.size();
        // legoloam.IP_->laserCloudIn->height = 1;

        return true;
	}

    /*
    if (s->FrameSensor == lidar_sensor) {
        std::stringstream tmp_filename;
        tmp_filename << std::setw(10) << std::setfill('0') << frame_id;
        std::string lidar_file_pcd = tmp_filename.str() + ".pcd";
        lidar_file_pcd = dirname + lidar_file_pcd;
        std::cout << lidar_file_pcd << std::endl;
        frame_id++;

        pcl::io::loadPCDFile(lidar_file_pcd, *(legoloam.IP_->laserCloudIn));

        legoloam.IP_->laserCloudInMetadata.timestamp = frame_id;

        return true;
    } */
	
	return false;
}


bool sb_process_once(SLAMBenchLibraryHelper *slam_settings) {
    
    legoloam.Run();

    if (legoloam.MO_->cloudKeyPoses6D->points.size() == 0) {
        pose = Eigen::Matrix4f::Identity();
        return true;
    }

    PointXYZIRPYT pose6d = legoloam.MO_->cloudKeyPoses6D->points.back();

    Eigen::AngleAxisf rollAngle(pose6d.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pose6d.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(pose6d.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotationMatrix = (yawAngle * pitchAngle * rollAngle).matrix();

    // Create the translation vector
    Eigen::Vector3f translation(pose6d.x, pose6d.y, pose6d.z);

    // Combine the rotation matrix and translation vector into a Matrix4f pose
    pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = rotationMatrix;
    pose.block<3, 1>(0, 3) = translation;

    return true;
}


bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p) {
    (void)lib;

    slambench::TimeStamp ts = *ts_p;

    if (pose_output->IsActive()) {
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		pose_output->AddPoint(ts, new slambench::values::PoseValue(align_mat * velo_2_lgrey * pose));
    }

    return true;
}


bool sb_clean_slam_system() {
    delete pose_output;
    delete pointcloud_output;
    delete lidar_sensor;
    return true;
}
