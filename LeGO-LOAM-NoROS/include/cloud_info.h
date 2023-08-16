#pragma once

#include <vector>
#include <iostream>

namespace cloud_msgs
{
    struct cloud_info
    {   
        double timestamp;
        std::vector<int32_t> startRingIndex;
        std::vector<int32_t> endRingIndex;
        float startOrientation = 0.0;
        float endOrientation = 0.0;
        float orientationDiff = 0.0;
        std::vector<uint8_t> segmentedCloudGroundFlag;
        std::vector<uint32_t> segmentedCloudColInd;
        std::vector<float> segmentedCloudRange;

        cloud_info() = default;

        friend std::ostream &operator<<(std::ostream &s, const cloud_info &v)
        {
            s << "cloud_info_(";
            s << "startRingIndex: " << v.startRingIndex.size();
            s << ", endRingIndex: " << v.endRingIndex.size();
            s << ", startOrientation: " << v.startOrientation;
            s << ", endOrientation: " << v.endOrientation;
            s << ", orientationDiff: " << v.orientationDiff;
            s << ", segmentedCloudGroundFlag: " << v.segmentedCloudGroundFlag.size();
            s << ", segmentedCloudColInd: " << v.segmentedCloudColInd.size();
            s << ", segmentedCloudRange: " << v.segmentedCloudRange.size();
            s << ")";
            return s;
        }

        friend bool operator==(const cloud_info &lhs, const cloud_info &rhs)
        {
            return lhs.startRingIndex == rhs.startRingIndex &&
                   lhs.endRingIndex == rhs.endRingIndex &&
                   lhs.startOrientation == rhs.startOrientation &&
                   lhs.endOrientation == rhs.endOrientation &&
                   lhs.orientationDiff == rhs.orientationDiff &&
                   lhs.segmentedCloudGroundFlag == rhs.segmentedCloudGroundFlag &&
                   lhs.segmentedCloudColInd == rhs.segmentedCloudColInd &&
                   lhs.segmentedCloudRange == rhs.segmentedCloudRange;
        }

        friend bool operator!=(const cloud_info &lhs, const cloud_info &rhs)
        {
            return !(lhs == rhs);
        }
    };
} // namespace cloud_msgs