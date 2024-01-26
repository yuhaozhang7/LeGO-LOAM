#pragma once

#include "utility.h"
#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptimization.h"
#include "transformFusion.h"

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace lego_loam {

class LeGOLOAM {
 public:
    LeGOLOAM() = default;

    bool Init();

    void Run();
    
    void Reset();
  
    std::unique_ptr<ImageProjection> IP_{nullptr};

    std::unique_ptr<FeatureAssociation> FA_{nullptr};

    std::unique_ptr<MapOptimization> MO_{nullptr};

    std::unique_ptr<TransformFusion> TF_{nullptr};

    std::mutex mutex;

    bool isOdomAftMapAvailable = false;
    bool isNewOdomAvailable = false;

    int process_count = 0;
    int processCountFromLastOdom = 0;

    PointCloudWithMetadata FAoutlierCloudMetadata;
    PointCloudWithMetadata FAlaserCloudCornerLastMetadata;
    PointCloudWithMetadata FAlaserCloudSurfLastMetadata;
    Odometry FAlaserOdometry;

    Odometry latestOdomAftMapped;
    Odometry final_odometry;

};

}  // namespace lego_loam