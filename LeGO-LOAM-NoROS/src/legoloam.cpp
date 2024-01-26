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
        while (true) {

            bool inputRecived = false;

            mutex.lock();
            if (FAoutlierCloudMetadata.frame_id == "/camera") {
                MO_->adjustLaserInput(FAoutlierCloudMetadata, FAlaserCloudCornerLastMetadata, FAlaserCloudSurfLastMetadata, FAlaserOdometry);
                inputRecived = true;
            }
            mutex.unlock();

            if (inputRecived) {
                MO_->process();

                mutex.lock();
                latestOdomAftMapped = MO_->odomAftMapped;
                isOdomAftMapAvailable = true;
                processCountFromLastOdom = 0;
                mutex.unlock();
            }

        }
    });
    moThread.detach();


    return true;
}

void LeGOLOAM::Run() {
    
    std::cout << "Prcoess Frame " << process_count + 1 << std::endl;

    IP_->process();

    FA_->adjustLaserInput(IP_->segMsg, IP_->segmentedCloudMetadata, IP_->outlierCloudMetadata);
    FA_->process();

    /*
    if (process_count % 5 == 0) {
        MO_->adjustLaserInput(FA_->outlierCloudMetadata, FA_->laserCloudCornerLastMetadata, FA_->laserCloudSurfLastMetadata, FA_->laserOdometry);
        MO_->process();
    }

    TF_->odomAftMappedHandler(MO_->odomAftMapped);
    TF_->laserOdometryHandler(FA_->laserOdometry);
    */
    
    if (process_count % 1 == 0) {
        
        mutex.lock();

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

        mutex.unlock();

    }

    while (true) {
        mutex.lock();
        if (processCountFromLastOdom < 2 && isOdomAftMapAvailable) break;
        mutex.unlock();
    }
    mutex.unlock();

    mutex.lock();
    TF_->odomAftMappedHandler(latestOdomAftMapped);
    TF_->laserOdometryHandler(FA_->laserOdometry);
    processCountFromLastOdom++;
    mutex.unlock();

    IP_->resetParameters();

    process_count++;

}

}