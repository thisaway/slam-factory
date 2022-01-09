#ifndef SLAM_FACTORY_FEATURE_FEATURE_H
#define SLAM_FACTORY_FEATURE_FEATURE_H

#include "Core/core.h"
#include "Types.h"

namespace sf{

    class SF_EXPORTS Feature{
    public:
        virtual ~Feature();
    };

    class SF_EXPORTS Feature2d:public Feature{
    public:
        virtual ~Feature2d();

        /** @brief Detect feature points in the region of interest of the input image；
        *** @param img Image to be detected;
        *** @param keypoints Keypoints detected;
        *** @param box Region of interest,box=(-1,-1,-1,-1) represents the detection of the entire image;
        **/
        virtual void detectKeypoints(SF_IN Image& img,
                                     SF_OUT std::vector<Keypoints>& keypoints,
                                     Box box=(-1,-1,-1,-1));
        
        /** @brief Compute feature descriptors of corresponding keypoints；
        *** @param img Image to be detected;
        *** @param keypoints Keypoints detected;
        *** @param descriptors Descriptors computed;
        **/
        virtual void computeDescriptors(SF_IN Image& img,
                                        SF_IN std::vector<Keypoints>& keypoints,
                                        SF_OUT std::vector<Descriptor>& descriptors);

        /** @brief Detect feature points and compute feature descriptors of corresponding keypoints  \
        *** in the region of interest of the input image；
        *** @param img Image to be detected;
        *** @param keypoints Keypoints detected;
        *** @param descriptors Descriptors computed;
        *** @param box Region of interest,box=(-1,-1,-1,-1) represents the detection of the entire image;
        **/
        virtual void detectAndCompute(SF_IN Image& img,
                                      SF_OUT std::vector<Keypoints>& keypoints,
                                      SF_OUT std::vector<Descriptor>& descriptors,
                                      Box box=(-1,-1,-1,-1))=0;
        
        /** @brief Get number of features;
        **/
        virtual uint32_t getFeatureNum() const=0;

        /** @brief Get size of each feature;
        **/
        virtual uint16_t getFeatureSize() const=0;
    };

    class SF_EXPORTS Orb:public Feature2d{
    public:
        enum{FAST_TYPE=0，HARRIS_TYPE=1};
        virtual ~ORB();

        /** @brief Orb constructor;
        *** @param featureNum Maximum number of reserved  orb features;
        *** @param scaleFactor For build pyramid;
        *** @param pyramidLevalNum Number of pyramid levels;
        *** @param srcImageLevel The level of source image in pyramid;
        *** @param adjacentAreaRadius Radius of keypoint adjacent area;
        *** @param randomPointNum Number of random point;
        *** @param keypointType Type of keypoint detection algorithm;
        *** @param featurePatchSize Size of each orb feature;
        *** @param fastThreshold Fast algorithm threshold;
        **/
        virtual static Orb* createOrbPtr(uint32_t featureNum=300,float scaleFactor=1.2f,  \
                                         uint8_t pyramidLevelNum=5,uint8_t srcImageLevel=1,  \
                                         float adjacentAreaRadius=31,uint8_t randomPointNum=2,  \
                                         uint8_t keypointType=FAST_TYPE,uint16_t featurePatchSize=31,  \
                                         float fastThreshold=20);

        virtual uint32_t getFeatureMaxNum() const=0;
        virtual void setFeatureMaxNum(uint32_t _featureNum)=0;

        virtual float getScaleFactor() const=0;
        virtual void setScaleFactor(float _scaleFactor)=0;

        virtual uint8_t getPyramidLevelNum() const=0;
        virtual void setPyramidLevelNum(uint8_t _pyramidLevelNum)=0;

        virtual uint8_t getSrcImageLevel() const=0;
        virtual void setSrcImageLevel(uint8_t _srcImageLevel)=0;

        virtual float getAdjacentAreaRadius() const=0;
        virtual void setAdjacentAreaRadius(float _adjacentAreaRadius)=0;

        virtual uint8_t getRandomPointNum() const=0;
        virtual void setRandomPointNum(uint8_t _randomPointNum)=0;

        virtual uint8_t getKeypointType() const=0;
        virtual void setKeypointType(uint8_t _keypointType)=0;

        virtual uint16_t getFeaturePatchSize(uint16_t _featurePatchSize) const=0;
        virtual void setFeaturePatchSize()=0;

        virtual float getFastThreshold() const=0;
        virtual void setFastThereshold(float _fastThreshold)=0;
    };
}