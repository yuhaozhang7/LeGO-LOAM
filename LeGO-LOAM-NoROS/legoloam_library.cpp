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
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>

#include "include/legoloam.h"

const std::string default_yaml_path = "/deps/legoloam/configs/configs.yaml";

// Parameters
std::string yaml_path;
bool show_point_cloud;
int point_cloud_ratio;
// extern parameters in utility.h
int N_SCAN;
int Horizon_SCAN;
float ang_res_x;
float ang_res_y;
float ang_bottom;
int groundScanInd;
float segmentAlphaX;
float segmentAlphaY;
int skipFrameNum;
int odo2Map;
double mappingProcessInterval;
std::string dataset_name;

// Sensors
slambench::io::LidarSensor *lidar_sensor;

slambench::TimeStamp last_frame_timestamp;
double current_timestamp;
double fake_timestamp = 1000.0;

int frame_count = 0;

// Outputs
slambench::outputs::Output *legoloam_pose_output;
slambench::outputs::Output *legoloam_pointcloud_output;

// System
static lego_loam::LeGOLOAM legoloam;
// contains rotation only
Eigen::Matrix4f align_mat = (Eigen::Matrix4f() << -1.0,  0.0, 0.0, 0.0,
                                                   0.0, -1.0, 0.0, 0.0,
                                                   0.0,  0.0, 1.0, 0.0,
                                                   0.0,  0.0, 0.0, 1.0).finished();
Eigen::Matrix4f pose;


bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {

    slam_settings->addParameter(TypedParameter<std::string>("configs", "configuration", "path to configuration YAML file", &yaml_path, &default_yaml_path));

    return true;
}


bool sb_init_slam_system(SLAMBenchLibraryHelper *slam_settings) {
    
    // Declare Outputs
    legoloam_pose_output = new slambench::outputs::Output("LeGO-LOAM Pose", slambench::values::VT_POSE, true);

    legoloam_pointcloud_output = new slambench::outputs::Output("LeGO-LOAM PointCloud", slambench::values::VT_POINTCLOUD, true);
    legoloam_pointcloud_output->SetKeepOnlyMostRecent(true);

    slam_settings->GetOutputManager().RegisterOutput(legoloam_pose_output);
    slam_settings->GetOutputManager().RegisterOutput(legoloam_pointcloud_output);

    // Inspect sensors
    lidar_sensor = (slambench::io::LidarSensor*)slam_settings->get_sensors().GetSensor(slambench::io::LidarSensor::kLidarType);
    if (lidar_sensor == nullptr) {
        std::cerr << "Invalid sensors found, Lidar not found." << std::endl;
        return false;
    }

    // ================================Read YAML================================
    YAML::Node config = YAML::LoadFile(yaml_path);

    N_SCAN = config["N_SCAN"].as<int>();
    Horizon_SCAN = config["Horizon_SCAN"].as<int>();
    ang_res_x = config["ang_res_x"].as<float>();
    ang_res_y = config["ang_res_y"].as<float>();
    ang_bottom = config["ang_bottom"].as<float>();
    groundScanInd = config["groundScanInd"].as<int>();
    segmentAlphaX = ang_res_x / 180.0 * M_PI;
    segmentAlphaY = ang_res_y / 180.0 * M_PI;
    odo2Map = config["odo_2_map"].as<int>();
    skipFrameNum = config["skip_frame_num"].as<int>();
    mappingProcessInterval = config["mapping_process_interval"].as<double>();
    dataset_name = config["dataset_name"].as<std::string>();

    show_point_cloud = config["show_point_cloud"].as<bool>();
    point_cloud_ratio = config["point_cloud_ratio"].as<int>();
    
    std::cout << "N_SCAN: " << N_SCAN << std::endl;
    std::cout << "Horizon_SCAN: " << Horizon_SCAN << std::endl;
    std::cout << "ang_res_x: " << ang_res_x << std::endl;
    std::cout << "ang_res_y: " << ang_res_y << std::endl;
    std::cout << "ang_bottom: " << ang_bottom << std::endl;
    std::cout << "groundScanInd: " << groundScanInd << std::endl;
    std::cout << "skipFrameNum: " << skipFrameNum << std::endl;
    std::cout << "mappingProcessInterval: " << mappingProcessInterval << std::endl;
    std::cout << "dataset_name: " << dataset_name << std::endl;
    std::cout << "show_point_cloud: " << show_point_cloud << std::endl;
    std::cout << "point_cloud_ratio: " << point_cloud_ratio << std::endl;

    // ================================Start LeGO-LOAM================================
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

        void* rawData = s->GetData();
        size_t dataSize = s->GetVariableSize(); // Assuming you have such a method. If not, you'd need another way to know the size.

        char* byteData = reinterpret_cast<char*>(rawData);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

        for (size_t i = 0; i < dataSize; i += sizeof(pcl::PointXYZI)) {
            pcl::PointXYZI point = *reinterpret_cast<pcl::PointXYZI*>(byteData + i);
            point.intensity = 0.0;
            legoloam.IP_->laserCloudIn->points.push_back(point);
        }

        legoloam.IP_->laserCloudIn->width = legoloam.IP_->laserCloudIn->points.size();
        legoloam.IP_->laserCloudIn->height = 1;

        return true;
	}
	
	return false;
}


bool sb_process_once(SLAMBenchLibraryHelper *slam_settings) {
    
    legoloam.Run();

    /*
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
    */

    if (frame_count == 0) {
        pose = Eigen::Matrix4f::Identity();
        frame_count++;
        return true;
    }

    Odometry final_odometry = legoloam.TF_->laserOdometry2;

    Eigen::Quaternionf quat(final_odometry.orientationW, final_odometry.orientationX, final_odometry.orientationY, final_odometry.orientationZ);
    Eigen::Matrix3f rotationMatrix = quat.toRotationMatrix();

    pose = Eigen::Matrix4f::Identity();

    pose.block<3, 3>(0, 0) = rotationMatrix;
    pose(0, 3) = final_odometry.positionX;
    pose(1, 3) = final_odometry.positionY;
    pose(2, 3) = final_odometry.positionZ;

    frame_count++;
    return true;
}


bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p) {
    (void)lib;

    slambench::TimeStamp ts = *ts_p;

    if (legoloam_pose_output->IsActive()) {
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		legoloam_pose_output->AddPoint(ts, new slambench::values::PoseValue(align_mat * pose));
    }
    
    if (legoloam_pointcloud_output->IsActive() && show_point_cloud) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_trans(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::transformPointCloud(*(legoloam.MO_->cloudOutMetadata.cloud), *cloud_out_trans, align_mat);
        // pcl::transformPointCloud(*(legoloam.FA_->laserCloudCornerLastMetadata.cloud), *cloud_out_trans, align_mat);

        auto slambench_point_cloud = new slambench::values::PointCloudValue();
        int count = 0;
        for(const auto &p : *cloud_out_trans) {
            if (count % point_cloud_ratio == 0) slambench_point_cloud->AddPoint(slambench::values::Point3DF(p.x, p.y, p.z));
            count++;
        }

        // Take lock only after generating the map
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        legoloam_pointcloud_output->AddPoint(ts, slambench_point_cloud);
    }

    return true;
}


bool sb_clean_slam_system() {
    delete legoloam_pose_output;
    delete legoloam_pointcloud_output;
    delete lidar_sensor;
    return true;
}
