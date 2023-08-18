#pragma once

#include <yaml-cpp/node/node.h>
#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptimization.h"

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
};

}  // namespace lego_loam