#ifndef SLAM_FACTORY_FEATURE_DESCRIPTOR
#define SLAM_FACTORY_FEATURE_DESCRIPTOR

#include "Core/Core.h"

namespace sf{
    class Descriptor{
        /** @brief Compute description of keypoints;
        *** @param img Input image;
        *** @param keypoints Keypoints to be described;
        *** @param descriptions Description of keypoints;
        **/
        virtual void computeDescription(SF_IN const Image& img,
                                        SF_IN const std::vector<Keypoint*>& keypoints,
                                        SF_OUT std::vector<Description>& descriptions)=0;
    };
}