#include <legoloam.h>

namespace lego_loam{

bool LeGOLOAM::Init() {
    process_count = 0;
    IP_ = std::make_unique<ImageProjection>();
    FA_ = std::make_unique<FeatureAssociation>();
    MO_ = std::make_unique<MapOptimization>();
    TF_ = std::make_unique<TransformFusion>();

    FAoutlierCloudMetadata.cloud.reset(new pcl::PointCloud<PointType>());
    FAlaserCloudCornerLastMetadata.cloud.reset(new pcl::PointCloud<PointType>());
    FAlaserCloudSurfLastMetadata.cloud.reset(new pcl::PointCloud<PointType>());

    // Create a thread for MO_ processing
    std::thread moThread([this] {
        while (true) { // Replace with a proper condition for thread termination
            std::unique_lock<std::mutex> lock(moMutex);
            moCondition.wait(lock, [this]{ return moUpdateFlag; });

            MO_->adjustLaserInput(FAoutlierCloudMetadata, FAlaserCloudCornerLastMetadata, FAlaserCloudSurfLastMetadata, FAlaserOdometry);
            MO_->process();

            // Store the latest output of MO_ in a buffer
            std::lock_guard<std::mutex> resultLock(resultMutex);
            latestOdomAftMapped = MO_->odomAftMapped;
            moUpdateFlag = false;
            
            isNewOdomAvailable = true;
            processCountForCurrentOdom = 0;
            newOdomAvailable.notify_one();
        }
    });
    moThread.detach();


    return true;
}

void LeGOLOAM::Run() {
    
    IP_->process();

    FA_->adjustLaserInput(IP_->segMsg, IP_->segmentedCloudMetadata, IP_->outlierCloudMetadata);
    FA_->process();

    // MO_->adjustLaserInput(FA_->outlierCloudMetadata, FA_->laserCloudCornerLastMetadata, FA_->laserCloudSurfLastMetadata, FA_->laserOdometry);
    // MO_->process();

    if (process_count % 5 == 0) {

        *(FAoutlierCloudMetadata.cloud) = *(FA_->outlierCloud);
        FAoutlierCloudMetadata.timestamp = FA_->timeScanCur;
        FAoutlierCloudMetadata.frame_id = "/camera";

        *(FAlaserCloudCornerLastMetadata.cloud) = *(FA_->laserCloudCornerLast);
        FAlaserCloudCornerLastMetadata.timestamp = FA_->timeScanCur;
        FAlaserCloudCornerLastMetadata.frame_id = "/camera";

        *(FAlaserCloudSurfLastMetadata.cloud) = *(FA_->laserCloudSurfLast);
        FAlaserCloudSurfLastMetadata.timestamp = FA_->timeScanCur;
        FAlaserCloudSurfLastMetadata.frame_id = "/camera";

        FAlaserOdometry = FA_->laserOdometry;

        {
            std::lock_guard<std::mutex> lock(moMutex);
            moUpdateFlag = true;
            moCondition.notify_one();
        }
    }

    {
        std::unique_lock<std::mutex> lock(resultMutex);
        while (!isNewOdomAvailable && process_count != 0) {
            newOdomAvailable.wait(lock); // Wait until a new odom is available
        }

        if (process_count != 0) {
            TF_->odomAftMappedHandler(latestOdomAftMapped);
            TF_->laserOdometryHandler(FA_->laserOdometry);
            processCountForCurrentOdom++;

            if (processCountForCurrentOdom >= 5) {
                isNewOdomAvailable = false; // Set to false to wait for new data
            }
        }
    }

    /*
    {
        // Use the latest output of MO_
        std::lock_guard<std::mutex> lock(resultMutex);
        if (process_count != 0) {
            TF_->odomAftMappedHandler(latestOdomAftMapped);
            TF_->laserOdometryHandler(FA_->laserOdometry);
        }
    }*/

    IP_->resetParameters();

    process_count++;

}

}