//Descriptor.h

#ifndef SLAM_FACTORY_FEATURE_DESCRIPTOR_H
#define SLAM_FACTORY_FEATURE_DESCRIPTOR_H

#include "Core.h"
#include "Keypoint.h"
#include "Types.h"

#include <vector>

namespace sf{

class Image;

template <typename _Tp, int _Length>
class SF_EXPORTS Descriptor{

public:
    
    Descriptor(){}
    ~Descriptor(){}

    /** @brief Compute description of keypoints;
    *** @param img Input image;
    *** @param keypoints Keypoints to be described;
    *** @param descriptions Description of keypoints;
    **/
    virtual void computeDescription(SF_IN const Image& img,  \
            SF_IN const std::vector<std::vector<Keypoint>>& keypoints,  \
            SF_OUT std::vector<std::vector<Description<_Tp, _Length>>>& descriptions) = 0; 
};

template <int _Length>
class SF_EXPORTS BriefDescriptor : public Descriptor<BriefMetaType, _Length>{

public:

    virtual ~BriefDescriptor(){}

    enum{UNIFORM = 1, NORMAL = 2};

    #define BRIEF_BIT_EACH_INT 8
    
    static SharedPtr<BriefDescriptor> createDescriptorPtr(int numComparedPoints = 2,  \
            int descriptionLength = 256, int adjacentAreaRadius = 15, int randomType = UNIFORM);

    virtual int getAdjacentAreaRadius() const = 0;
    virtual void setAdjacentAreaRadius(int adjacentAreaRadius) = 0;

    virtual int getNumComparedPoints() const = 0;
    virtual void setNumComparedPoints(int numComparedPoints) = 0;
    
    virtual int getDescriptionLength() const = 0;
    virtual void setDescriptionLength(int descriptionLength) = 0;

    virtual int getRandomType() const = 0;
    virtual void setRandomType(int randomType) = 0;
};


}  //namespace

#endif