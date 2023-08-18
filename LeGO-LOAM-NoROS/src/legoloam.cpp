#include <legoloam.h>

namespace lego_loam{

bool LeGOLOAM::Init() {
    IP_ = std::make_unique<ImageProjection>();
    FA_ = std::make_unique<FeatureAssociation>();
    MO_ = std::make_unique<MapOptimization>();
    return true;
}

void LeGOLOAM::Run() {
    
    IP_->process();

    FA_->adjustLaserInput(IP_->segMsg, IP_->segmentedCloudMetadata, IP_->outlierCloudMetadata);
    FA_->process();

    MO_->adjustLaserInput(FA_->outlierCloudMetadata, FA_->laserCloudCornerLastMetadata, FA_->laserCloudSurfLastMetadata, FA_->laserOdometry);
    MO_->process();

    IP_->resetParameters();

}

}