//Detector.cc
#include "Detector.h"
#include "Image.h"

#include <cassert>
#include <cmath>
#include <memory>
#include <iostream>


namespace sf{

struct DetectorIn{

    uint16_t nmsR;  //NMS radius;
    uint16_t gr;  //grid rows;
    uint16_t gc;  //grid cols;
};

const char* Detector::
getClassName() const{

    return "Detector";
}


using MatXpt2d = Eigen::Matrix<Point2d, Dynamic, Dynamic>;
MatXi32 kpMarks;  //A mat place wheather there is a keypoint;


#define FAST_CORNER_TEST(x, i)  \
{  \
    diff = pd - x;  \
    if(diff > 0){  \
        resultArray[i] = 1;  \
        if(i < fastType)resultArray[i + 16] = 1;  \
        sumGreat += diff;  \
    }  \
    else{  \
        diff = x - ps;  \
        if(diff >= 0){  \
            resultArray[i] = 2;  \
            if(i < fastType)resultArray[i + 16] = 2;  \
            sumSmall += diff;  \
        }  \
    }  \
}


/** @brief Compute orientation of a keypoint;
*** @param img Input image;
*** @param x The x coordinate of the upper left corner of area；
*** @param y The y coordinate of the upper left corner of area；
*** @param h Height of the area;
*** @param w Width of the area;
*** @param orientation Orientation to be computed;
**/
static void computeOrientation(const Image& img, uint32_t x, uint32_t y,  \
        uint32_t h, uint32_t w, double& orientation){

    uint32_t bottomRightX = x + h, bottomRightY = y + w;

    assert(bottomRightX < img.rows and bottomRightY < img.cols);

    double xpSum = 0, ypSum = 0;

    for(; x < bottomRightX; ++x){
        for(; y < bottomRightY; ++y){

            xpSum += x * img(x, y);
            ypSum += y * img(x, y);
        }
    }

    orientation=atan(ypSum / double(xpSum));
}


//If (x,y) point is suppression,return false;otherwise return true;
static bool nms(const uint32_t& x, const uint32_t& y, const uint16_t& nmsRadius, const double& score,  \
        std::vector<Keypoint>& keypoints, uint32_t& nkps){

    for(uint32_t xi = x - nmsRadius; xi <= x; ++xi){
        for(uint32_t yi = y - nmsRadius; yi <= y; ++yi){

            if(kpMarks(xi, yi) < 0 or keypoints[kpMarks(xi, yi)].bad)continue;
            if(keypoints[kpMarks(xi, yi)].score >= score)return false;
            else{
                keypoints[kpMarks(xi, yi)].bad = true;
                --nkps;
            }
        }
    }
    return true;
}


//Detector
void Detector::
gridAssignment(uint32_t rows, uint32_t cols){
    
    internalPtr->gr = rows;
    internalPtr->gc = cols;
}


bool Detector::
gridUsed() const{

    return internalPtr->gr != 0 and internalPtr->gc !=0 ;
}

void Detector::
setNms(uint16_t radius){

    internalPtr->nmsR = radius;
}


bool Detector::
nmsUsed() const{

    return internalPtr->nmsR > 0;
}


static bool compareKpScore(const Keypoint& kp1, const Keypoint& kp2){
    return kp1.score > kp2.score;
}


static void retainMaximunKps(uint32_t mnkps, std::vector<Keypoint>& keypoints, size_t ibeg, size_t iend){
    assert(iend <= keypoints.size());

    if(ibeg >= iend)return;

    sort(keypoints.begin() + ibeg, keypoints.begin() + iend, compareKpScore);

    uint32_t num = 0;
    size_t k = 0, kend = keypoints.size();
    for(; k < kend; ++k){

        if(num == mnkps){

            keypoints[k].bad = true;
            continue;
        }

        if(keypoints[k].bad == false)++num;
    }
    keypoints.shrink_to_fit();
}


//FastDetector
class FastDetectorImpl SF_FINAL : public FastDetector{

public:

    FastDetectorImpl(uint32_t maxNumKeypoints, float pixelThreshold, uint8_t fastType,  \
        float adjacentAreaRadius, uint8_t octave, uint16_t nmsRadius = 0, uint16_t gridRows = 0,  \
        uint16_t gridCols = 0) : mnkps(maxNumKeypoints), pTh(pixelThreshold), areaR(adjacentAreaRadius),  \
        oct(octave), type(fastType), nkps(0){
             
            internalPtr = new DetectorIn();
            internalPtr -> nmsR = nmsRadius; 
            internalPtr -> gr = gridRows; 
            internalPtr -> gc = gridCols;            
        }
    
    ~FastDetectorImpl(){delete internalPtr;}
    
    uint32_t getMaxNumKeypoints() const SF_OVERRIDE{return mnkps;}
    void setMaxNumKeypoints(uint32_t maxNumKeypoints) SF_OVERRIDE {mnkps = maxNumKeypoints;}

    float getAdjacentAreaRadius() const SF_OVERRIDE{return areaR;}
    void setAdjacentAreaRadius(float radius) SF_OVERRIDE{areaR = radius;}

    uint8_t getOctave() const SF_OVERRIDE{return oct;}
    void setOctave(uint8_t octave) SF_OVERRIDE{oct = octave;}

    float getPixelThreshold() const SF_OVERRIDE{return pTh;}
    void setPixelThreshold(float pixelThreshold) SF_OVERRIDE {pTh = pixelThreshold;}

    uint8_t getFastType() const SF_OVERRIDE{return type;}
    void setFastType(uint8_t fastType) SF_OVERRIDE{type = fastType;}

    uint32_t getNumKeypoints() const SF_OVERRIDE{return nkps;}

    void detectKeypoints(const Image& img, uint32_t x, uint32_t y, uint32_t height,  \
                            uint32_t width, std::vector<Keypoint>& keypoints) SF_OVERRIDE;

private:

    void detectSubAreaKeypoints(const Image& img, uint32_t x, uint32_t y,  \
        uint32_t h, uint32_t w, std::vector<Keypoint>& keypoints,  \
        uint32_t& kpId, uint32_t& nkps);

    bool isFastCorner(const Image& img, const uint32_t& x, const uint32_t& y,  \
        const uint8_t& fastType, const float& pixelTh, double& score);

protected:

    uint32_t mnkps;  //max num keypoints;
    float pTh;  //pixel threshold;
    float areaR;
    uint8_t oct;  //octave
    uint8_t type;  //FAST4 or FAST9 or FAST12;
    uint32_t nkps;  //keypoints number after NMS;
};


void FastDetectorImpl::
detectKeypoints(const Image& img, uint32_t x, uint32_t y, uint32_t height,  \
        uint32_t width, std::vector<Keypoint>& keypoints){

    assert(x > 4 and y > 4);
    assert(x + height < img.rows-3 and y + width < img.cols-3);

    kpMarks = MatXi32::Ones(img.rows, img.cols);  //TODO
    kpMarks *= -1;

    keypoints.reserve(2 * mnkps);

    uint32_t kpId = 0;

    uint32_t gr = internalPtr->gr;
    uint32_t gc = internalPtr->gc;

    if(gr == 0 or gc == 0){  //No assign grid;

        detectSubAreaKeypoints(img, x, y, height, width, keypoints, kpId, nkps);
        if(nkps > mnkps)retainMaximunKps(mnkps, keypoints, 0, keypoints.size());
    }
    else{  //assigned grid;

        uint32_t gmnkps = mnkps / (gr * gc);

        uint32_t gridCellHeight = height / gr, gridCellWidth = width / gc;

        std::vector<size_t> eachGridKps;
        eachGridKps.reserve(gr * gc);
        eachGridKps.push_back(0);
        size_t lastKps = 0;
        uint32_t nkpsLast;

        for(uint32_t gridx = 0; gridx < gr - 1; ++gridx){
            for(uint32_t gridy = 0; gridy < gc - 1; ++gridy){
                
                nkpsLast = nkps;
                detectSubAreaKeypoints(img, (gridx + 1) * gridCellHeight, (gridy + 1) * gridCellWidth,  \
                    gridCellHeight, gridCellWidth, keypoints, kpId, nkps);
                
                if(nkps - nkpsLast <= gmnkps)eachGridKps.push_back(lastKps);
                else eachGridKps.push_back(keypoints.size() - lastKps);

                lastKps = keypoints.size();
            }
        }

        for(size_t i = 0, iend = eachGridKps.size() - 1; i < iend; ++i){

            retainMaximunKps(gmnkps, keypoints, eachGridKps[i], eachGridKps[i+1]);
        }
    }
}

void FastDetectorImpl::
detectSubAreaKeypoints(const Image& img, uint32_t x, uint32_t y,  \
        uint32_t h, uint32_t w, std::vector<Keypoint>& keypoints,  \
        uint32_t& kpId, uint32_t& nkps){

    uint16_t nmsRadius = internalPtr -> nmsR;

    double orientation, score;

    size_t n = keypoints.size();

    for(uint32_t xi = x, xend = x + h; xi < xend; ++xi){
        //std::cout<<"corner test:"<<x<<","<<y<<std::endl;
        for(uint32_t yi = y, yend = y + w; yi < yend; ++yi){
            
            if(!isFastCorner(img, xi, yi, type, pTh, score))continue;
            if(nmsRadius > 0 and !nms(xi, yi, nmsRadius, score, keypoints, nkps))continue;

            computeOrientation(img, xi, yi, 2 * areaR, 2 * areaR, orientation);
            
            keypoints.push_back(Keypoint(xi, yi, areaR, oct, orientation, score, kpId++, false));
            kpMarks(xi, yi)=n++;
            ++nkps;
        }
    }
}


bool FastDetectorImpl::
isFastCorner(const Image& img, const uint32_t& x, const uint32_t& y,  \
        const uint8_t& fastType, const float& pixelTh, double& score){

    uint8_t resultArray[16 + fastType] = {0};
    double sumSmall = 0.0, sumGreat = 0.0;

    const double pd = img(x, y) - pixelTh;
    const double ps = img(x, y) + pixelTh;

    double diff;

    FAST_CORNER_TEST(img(x - 4, y), 0)  //0
    FAST_CORNER_TEST(img(x + 4, y), 8)  //8

    uint8_t d = resultArray[0] | resultArray[8];

    if(d == 0)return false;

    //std::cout<<"0&&8 corner test:"<<x<<","<<y<<std::endl;

    FAST_CORNER_TEST(img(x, y + 4), 4)  //4
    FAST_CORNER_TEST(img(x, y - 4), 12)  //12

    d &= resultArray[4] | resultArray[12];

    if(d == 0)return false;

    //std::cout<<"4&&12 corner test:"<<x<<","<<y<<std::endl;

    if(fastType == FastDetector::FAST4){
        if(d & 1){
            score = sumGreat;
            return true;
        }
        else if(d & 2){
            score = sumSmall;
            return true;
        }
        return false;
    }

    FAST_CORNER_TEST(img(x - 3, y + 3), 2);  //2
    FAST_CORNER_TEST(img(x + 3, y - 3), 10);  //10
    FAST_CORNER_TEST(img(x + 3, y + 3), 6);  //6
    FAST_CORNER_TEST(img(x - 3, y - 3), 14);  //14

    d &= resultArray[2] | resultArray[10];
    d &= resultArray[6] | resultArray[14];

    if(d == 0)return false;

    FAST_CORNER_TEST(img(x - 4, y + 1), 1);  //1
    
    FAST_CORNER_TEST(img(x - 1, y + 4), 3);  //3
    FAST_CORNER_TEST(img(x + 1, y + 4), 5);  //5
    
    FAST_CORNER_TEST(img(x + 4, y + 1), 7);  //7
    FAST_CORNER_TEST(img(x + 4, y - 1), 9);  //9
    
    FAST_CORNER_TEST(img(x + 1, y - 4), 11);  //11
    FAST_CORNER_TEST(img(x - 1, y - 4), 13);  //13
    
    FAST_CORNER_TEST(img(x + 4, y - 1), 15);  //15

    d &= resultArray[1] | resultArray[9];
    d &= resultArray[3] | resultArray[11];
    d &= resultArray[5] | resultArray[13];
    d &= resultArray[7] | resultArray[15];

    if(d == 0)return false;
    
    uint8_t count = 1;
    uint8_t i = 1,iend = 16 + fastType;
    for(; i < iend; ++i){
        if(count > fastType)break;
        if(resultArray[i] == resultArray[i-1])++count;
        else count = 1;
    }

    if(count > fastType){
        if(resultArray[i-1] == 1)score = sumGreat;
        else score = sumSmall;

        return true;
    }

    return false;
}


SharedPtr<FastDetector> FastDetector::
createDetectorPtr(uint32_t maxNumKeypoints, float pixelThreshold, uint8_t fastType,  \
        float adjacentAreaRadius, uint8_t octave, uint16_t nmsRadius, uint16_t gridRows,  \
        uint16_t gridCols){

    assert(fastType == FastDetector::FAST4 or fastType == FastDetector::FAST9 or  \
            fastType == FastDetector::FAST12);

    assert(pixelThreshold > 0);

    return std::make_shared<FastDetectorImpl>(maxNumKeypoints, pixelThreshold, fastType, adjacentAreaRadius,  \
                                            octave, nmsRadius, gridRows, gridCols);
}


const char* FastDetector::
getClassName() const{

    return "FastDetector";
}

}  //namespace