//Detector.cc
#include "Detector.h"
#include "Image.h"

#include <cassert>
#include <cmath>
#include <memory>


namespace sf{

struct DetectorIn{

    int nmsR;  //NMS radius;
    int gr;  //grid rows;
    int gc;  //grid cols;
};

const char* Detector::
getClassName() const{

    return "Detector";
}


using MatXpt2i = Eigen::Matrix<Point2i, Dynamic, Dynamic>;
MatXpt2i kpMarks;  //A mat place wheather there is a keypoint;


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
static void computeOrientation(const Image& img, int x, int y,  \
        int h, int w, double& orientation){

    int bottomRightX = x + h, bottomRightY = y + w;

    assert(bottomRightX < img.rows and bottomRightY < img.cols);

    double xpSum = 0, ypSum = 0;

    for(; x < bottomRightX; ++x){
        for(; y < bottomRightY; ++y){

            xpSum += x * img(x, y);
            ypSum += y * img(x, y);
        }
    }

    orientation=atan(ypSum / static_cast<double>(xpSum));
}


//If (x,y) point is suppression,return false;otherwise return true;
static bool nms(const int& x, const int& y, const int& nmsRadius, const double& score,  \
        std::vector<std::vector<Keypoint>>& keypoints){

    for(int xi = x - nmsRadius; xi <= x; ++xi){
        for(int yi = y - nmsRadius; yi <= y; ++yi){

            const Point2i& pt = kpMarks(xi, yi);

            if(pt.x < 0 or keypoints[pt.x][pt.y].score < 0)continue;

            if(keypoints[pt.x][pt.y].score >= score)return false;
            else{

                keypoints[pt.x][pt.y].score = -1;
            }
        }
    }
    return true;
}


//Detector
void Detector::
gridAssignment(int rows, int cols){
    
    assert(rows >= 0 and cols >= 0);

    internalPtr -> gr = rows;
    internalPtr -> gc = cols;
}


bool Detector::
gridUsed() const{

    return internalPtr -> gr != 0 and internalPtr -> gc != 0 ;
}

void Detector::
setNms(int radius){

    assert(radius >= 0);
    internalPtr -> nmsR = radius;
}


bool Detector::
nmsUsed() const{

    return internalPtr -> nmsR > 0;
}


static bool compareKpScore(const Keypoint& kp1, const Keypoint& kp2){
    
    return kp1.score > kp2.score;
}


static int serachFirstKeypointNegetive(const std::vector<Keypoint>& keypoints, double value){

    size_t ibeg = 0, iend = keypoints.size();
    size_t imid = (ibeg + iend) / 2;
    
    while(ibeg >= iend){
        
        if(keypoints[imid].score > value)iend = imid;
        else ibeg = imid;

        imid = (ibeg + iend) / 2;
    }

    return imid;
}


//FastDetector
class FastDetectorImpl SF_FINAL : public FastDetector{

public:

    FastDetectorImpl(uint32_t maxNumKeypoints, float pixelThreshold, int fastType,  \
        int adjacentAreaRadius, int octave, int nmsRadius = 0, int gridRows = 0,  \
        int gridCols = 0) : mnkps(maxNumKeypoints), pTh(pixelThreshold), areaR(adjacentAreaRadius),  \
        oct(octave), type(fastType), nkps(0){
             
            internalPtr = new DetectorIn();
            internalPtr -> nmsR = nmsRadius; 
            internalPtr -> gr = gridRows; 
            internalPtr -> gc = gridCols;            
        }
    
    ~FastDetectorImpl(){delete internalPtr;}
    
    inline uint32_t getMaxNumKeypoints() const SF_OVERRIDE{return mnkps;}
    inline void setMaxNumKeypoints(uint32_t maxNumKeypoints) SF_OVERRIDE {
        
        mnkps = maxNumKeypoints;
    }

    inline int getAdjacentAreaRadius() const SF_OVERRIDE{return areaR;}
    inline void setAdjacentAreaRadius(int radius) SF_OVERRIDE{
        
        assert(radius >= 0);
        areaR = radius;
    }

    inline int getOctave() const SF_OVERRIDE{return oct;}
    inline void setOctave(int octave) SF_OVERRIDE{oct = octave;}

    inline float getPixelThreshold() const SF_OVERRIDE{return pTh;}
    inline void setPixelThreshold(float pixelThreshold) SF_OVERRIDE {pTh = pixelThreshold;}

    inline int getFastType() const SF_OVERRIDE{return type;}
    inline void setFastType(int fastType) SF_OVERRIDE{type = fastType;}

    inline int getNumKeypoints() const SF_OVERRIDE{return nkps;}

    void detectKeypoints(const Image& img, int x, int y, int height,  \
            int width, std::vector<std::vector<Keypoint>>& keypoints) SF_OVERRIDE;

private:

    void detectSubAreaKeypoints(const Image& img, int x, int y,  \
        int h, int w, std::vector<std::vector<Keypoint>>& keypoints,  \
        int& kpId, size_t index);

    bool isFastCorner(const Image& img, const int& x, const int& y,  \
        const int& fastType, const float& pixelTh, double& score);
    
    void retainMaximunKps(uint32_t cmnkps, std::vector<Keypoint>& keypoints);

protected:

    uint32_t mnkps;  //max num keypoints;
    float pTh;  //pixel threshold;
    int areaR;
    int oct;  //octave
    int type;  //FAST4 or FAST9 or FAST12;
    int nkps;  //keypoints number after NMS;
};


void FastDetectorImpl::
retainMaximunKps(uint32_t cmnkps, std::vector<Keypoint>& keypoints){

    if(keypoints.size() == 0){

        keypoints.shrink_to_fit();
        return;
    }

    sort(keypoints.begin(), keypoints.end(), compareKpScore);

    if(internalPtr->nmsR > 0){

        int negetiveIndex = serachFirstKeypointNegetive(keypoints, -1);

        keypoints.resize(negetiveIndex);
    }
    else{

        if(cmnkps < keypoints.size())keypoints.resize(cmnkps);
    }

    keypoints.shrink_to_fit();
}


void FastDetectorImpl::
detectKeypoints(const Image& img, int x, int y, int height,  \
        int width, std::vector<std::vector<Keypoint>>& keypoints){

    assert(x > 4 and y > 4);
    assert(x + height < img.rows-3 and y + width < img.cols-3);

    kpMarks.resize(img.rows, img.cols);
    for(int r = 0; r < img.rows; ++r){
        for(int c = 0; c < img.cols; ++c){
            kpMarks(r, c) = Point2i32(-1, -1);
        }
    }

    int kpId = 0;

    int gr = internalPtr->gr;
    int gc = internalPtr->gc;

    if(gr == 0 or gc == 0){  //No assign grid;

        keypoints.resize(1);
        keypoints[0].reserve(2 * mnkps);

        detectSubAreaKeypoints(img, x, y, height, width, keypoints, kpId, 0);

        retainMaximunKps(mnkps, keypoints[0]);
        nkps += keypoints[0].size();
    }
    else{  //assigned grid;

        uint32_t gmnkps = mnkps / (gr * gc);

        int gridCellHeight = height / gr, gridCellWidth = width / gc;

        keypoints.resize(gr * gc);

        int kIndex;

        for(int gridx = 0; gridx < gr - 1; ++gridx){
            for(int gridy = 0; gridy < gc - 1; ++gridy){
                
                kIndex = gridx * (gc - 1) + gridy;
                keypoints[kIndex].reserve(2 * gmnkps);

                detectSubAreaKeypoints(img, (gridx + 1) * gridCellHeight, (gridy + 1) * gridCellWidth,  \
                    gridCellHeight, gridCellWidth, keypoints, kpId, kIndex);
            }
        }

        for(size_t i = 0, iend = keypoints.size(); i < iend; ++i){

            retainMaximunKps(gmnkps, keypoints[i]);
            nkps += keypoints[i].size();
        }
    }
}


void FastDetectorImpl::
detectSubAreaKeypoints(const Image& img, int x, int y,  \
        int h, int w, std::vector<std::vector<Keypoint>>& keypoints,  \
        int& kpId, size_t index){

    int nmsRadius = internalPtr -> nmsR;

    double orientation, score;

    for(int xi = x, xend = x + h; xi < xend; ++xi){

        for(int yi = y, yend = y + w; yi < yend; ++yi){
            
            if(!isFastCorner(img, xi, yi, type, pTh, score))continue;

            if(nmsRadius > 0 and !nms(xi, yi, nmsRadius, score, keypoints))continue;

            computeOrientation(img, xi, yi, 2 * areaR, 2 * areaR, orientation);
            
            keypoints[index].push_back(Keypoint(xi, yi, areaR, oct, orientation, score, kpId++));
            kpMarks(xi, yi) = Point2i32(index, keypoints[index].size() - 1);
        }
    }
}


bool FastDetectorImpl::
isFastCorner(const Image& img, const int& x, const int& y,  \
        const int& fastType, const float& pixelTh, double& score){

    int resultArray[16 + fastType] = {0};
    double sumSmall = 0.0, sumGreat = 0.0;

    const double pd = img(x, y) - pixelTh;
    const double ps = img(x, y) + pixelTh;

    double diff;

    FAST_CORNER_TEST(img(x - 4, y), 0)  //0
    FAST_CORNER_TEST(img(x + 4, y), 8)  //8

    int d = resultArray[0] | resultArray[8];

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
    
    int count = 1;
    int i = 1,iend = 16 + fastType;

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
createDetectorPtr(uint32_t maxNumKeypoints, float pixelThreshold, int fastType,  \
        int adjacentAreaRadius, int octave, int nmsRadius, int gridRows,  \
        int gridCols){

    assert(fastType == FastDetector::FAST4 or fastType == FastDetector::FAST9 or  \
            fastType == FastDetector::FAST12);

    assert(pixelThreshold > 0);
    assert(maxNumKeypoints > 0);
    assert(adjacentAreaRadius > 0);
    assert(nmsRadius >= 0);
    assert(gridRows >= 0 and gridCols >= 0);

    return std::make_shared<FastDetectorImpl>(maxNumKeypoints, pixelThreshold, fastType, adjacentAreaRadius,  \
                                            octave, nmsRadius, gridRows, gridCols);
}


const char* FastDetector::
getClassName() const{

    return "FastDetector";
}

}  //namespace