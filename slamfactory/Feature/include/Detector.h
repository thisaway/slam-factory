//Detector.h
#ifndef SLAM_FACTORY_FEATURE_DETECTOR_H
#define SLAM_FACTORY_FEATURE_DETECTOR_H

#include "Def.h"
#include "Keypoint.h"
#include "Types.h"

#include <vector>

namespace sf{
    
class Image;
struct DetectorIn;

class SF_EXPORTS Detector{

public:

    /** @brief Assign a grid to the image;
    *** @param rows Rows of grid;
    *** @param cols Cols of grid;
    **/
    void gridAssignment(int rows, int cols);
    
    /** @brief Is grid used;
    *** @return If rows > 0 and cols > 0 return true, otherwise return false;
    **/
    bool gridUsed() const;

    /** @brief Non-maximum suppression of detector;
    *** @param radius Radius of nms;
    **/
    void setNms(int radius);
    
    /** @brief Is NMS method used;
    *** @return Return true if nms used; otherwise,return false;
    **/
    bool nmsUsed() const;

    /** @brief Detect keypoints of image;
    *** @param img Image to be detected；
    *** @param x The x coordinate of the upper left corner of area to be detected；
    *** @param y The y coordinate of the upper left corner of area to be detected；
    *** @param height Height of the area to be detected；;
    *** @param width Width of the area to be detected；;
    *** @param keypoints Keypoints detected;
    **/
    virtual void detectKeypoints(SF_IN const Image& img, int x, int y, int height,  \
            int width, SF_OUT std::vector<std::vector<Keypoint>>& keypoints) = 0;

    virtual const char* getClassName() const;

    virtual ~Detector(){}

protected:

    DetectorIn* internalPtr;
};


class SF_EXPORTS FastDetector : public Detector{

public:

    enum {FAST4 = 3, FAST9 = 8, FAST12 = 11};

    /** @brief Create pointer for FastDetector;
    *** @param maxNumKeypoints Maximum number of keypoints;
    *** @param pixelThreshold The threshold of difference between pixels;
    *** @param fastType Type of FastDetector, FAST4 is fastest and FAST12 is most robust;
    *** @param adjacentAreaRadius Radius of adjacent area, FastDetector doesn't use that,  \
                                  just for building keypoint;
    *** @param octave Pyramid octave which the keypoint detected, FastDetector doesn't use that,  \
                      just for building keypoint;
    *** @param nmsRadius Radius of NMS, when nmsRadius > 0, means FastDetector will use NMS;
    *** @param gridRows Rows of grids, when gridRows > 0 and gridCols > 0, means FastDetector will use grid;
    *** @param gridCols Cols of grids, when gridRows > 0 and gridCols > 0, means FastDetector will use grid;
    **/
    static SharedPtr<FastDetector> createDetectorPtr(int maxNumKeypoints, float pixelThreshold,  \
            int fastType, int adjacentAreaRadius, int octave, int nmsRadius = 0,  \
            int gridRows = 0, int gridCols = 0);

    virtual int getMaxNumKeypoints() const = 0;
    virtual void setMaxNumKeypoints(int maxNumKeypoints) = 0;

    virtual float getPixelThreshold() const = 0;
    virtual void setPixelThreshold(float pixelThreshold) = 0;

    virtual int getAdjacentAreaRadius() const = 0;
    virtual void setAdjacentAreaRadius(int radius) = 0;

    virtual int getOctave() const = 0;
    virtual void setOctave(int octave) = 0;

    virtual int getFastType() const = 0;
    virtual void setFastType(int type) = 0;

    virtual int getNumKeypoints() const = 0;

    virtual const char* getClassName() const SF_OVERRIDE;

    virtual ~FastDetector(){}
};

}  //namespace

#endif