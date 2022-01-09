//testDetector.cc

#include "Detector.h"
#include "Image.h"
#include "Keypoint.h"
#include "Types.h"

#include "gtest/gtest.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace{

class TestDetector : public testing::Test{

protected:
    
    void SetUp() SF_OVERRIDE{

        img1 = sf::Image(100, 100, 0);
        img2 = sf::Image(300, 200, 10);
/*
        sf::MatXf mat1 = sf::MatXf::Random(100, 100);
        sf::MatXui8 mat2 = sf::MatXui8::Ones(100, 100);
        //sf::MatXf mat3 = (mat1 + mat2) * 127.5;  //[0, 255] scope;
        //mat3.cast<uint8_t>();
        img3 = sf::Image(mat2);
*/
        
        sf::Mat<uint8_t, 1, 20> vecc;
        vecc << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  \
               1, 1, 1, 1, 1, 1, 1, 1, 1, 1;

        sf::Mat<uint8_t, 20, 1> vecr;
        vecr << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  \
               1, 1, 1, 1, 1, 1, 1, 1, 1, 1;

        //45 degree angles;
        //1 0 0
        //0 1 0
        //1 1 1
        sf::MatXui8 corner1 = sf::MatXui8::Identity(20, 20);
        corner1.block(19, 0, 1, 20) = vecc;
        //std::cout << corner1(5, 5) << std::endl;
        corner1 *= 50;

        //1 1 1
        //0 1 0
        //0 0 1
        sf::MatXui8 corner2 = sf::MatXui8::Identity(20, 20);
        corner2.block(0, 0, 1, 20) = vecc;
        corner2 *= 50;

        //1 0 0
        //1 1 0
        //1 0 1
        sf::MatXui8 corner3 = sf::MatXui8::Identity(20, 20);
        corner3.block(0, 0, 20, 1) = vecr;
        corner3 *= 50;

        //1 1 1
        //0 1 0
        //0 0 1
        sf::MatXui8 corner4 = sf::MatXui8::Identity(20, 20);
        corner4.block(0, 0, 1, 20) = vecc;
        corner4 *= 50;

        //90 degree angles;
        //1 1 1
        //1 0 0
        //1 0 0
        sf::MatXui8 corner5 = sf::MatXui8::Zero(20, 20);
        corner5.block(0, 0, 1, 20) = vecc;
        corner5.block(0, 0, 20, 1) = vecr;
        corner5 *= 50;

        //1 0 0
        //1 0 0
        //1 1 1
        sf::MatXui8 corner6 = sf::MatXui8::Zero(20, 20);
        corner6.block(19, 0, 1, 20) = vecc;
        corner6.block(0, 0, 20, 1) = vecr;
        corner6 *= 50;


        sf::MatXui8 mat4 = sf::MatXui8::Zero(300, 300);
        mat4.block(5, 5, 20, 20) = corner3;  //yes
        mat4.block(200, 200, 20, 20) = corner3;  //yes
        mat4.block(100, 50, 20, 20) = corner2;  //yes
        img4 = sf::Image(mat4);  //2 keypoints
/*
        std::cout << img4(5,5) << std::endl;
        std::cout << img4(200,200) << std::endl;
*/

        sf::MatXui8 mat5 = sf::MatXui8::Zero(1000,1000);
        mat5.block(100, 100, 20, 20) = corner1;  //yes
        mat5.block(200, 200, 20, 20) = corner2;  //yes
        mat5.block(300, 300, 20, 20) = corner3;  //yes
        mat5.block(400, 400, 20, 20) = corner4;  //yes
        
        mat5.block(500, 500, 20, 20) = corner5;  //uncertain
        mat5.block(600, 600, 20, 20) = corner6;  //uncertain
        
        img5 = sf::Image(mat5);  //6 keypoints
    }

    sf::Image img1;
    sf::Image img2;
    //sf::Image img3;
    sf::Image img4;
    sf::Image img5;
};

TEST_F(TestDetector, Detector){

    sf::SharedPtr<sf::FastDetector> fastPtr = sf::FastDetector::createDetectorPtr(100, 30, 8, 30, 1);

    EXPECT_FALSE(fastPtr -> gridUsed());
    EXPECT_FALSE(fastPtr -> nmsUsed());

    fastPtr -> gridAssignment(10, 10);
    EXPECT_TRUE(fastPtr -> gridUsed());

    fastPtr -> setNms(3);
    EXPECT_TRUE(fastPtr -> nmsUsed());

    fastPtr -> gridAssignment(0, 0);
    EXPECT_FALSE(fastPtr -> gridUsed());

    fastPtr -> setNms(0);
    EXPECT_FALSE(fastPtr -> nmsUsed());
}


TEST_F(TestDetector, FastDetector){
    sf::SharedPtr<sf::FastDetector> fastPtr = sf::FastDetector::createDetectorPtr(100, 30, 8, 35, 1);

    EXPECT_EQ(fastPtr -> getMaxNumKeypoints(), 100);
    fastPtr -> setMaxNumKeypoints(50);
    EXPECT_EQ(fastPtr -> getMaxNumKeypoints(), 50);

    EXPECT_TRUE(abs(fastPtr -> getPixelThreshold() - 30) < 0.001);
    fastPtr -> setPixelThreshold(20);
    EXPECT_TRUE(abs(fastPtr -> getPixelThreshold() - 20) < 0.001);

    EXPECT_EQ(fastPtr -> getFastType(), sf::FastDetector::FAST9);
    fastPtr -> setFastType(sf::FastDetector::FAST12);
    EXPECT_EQ(fastPtr -> getFastType(), sf::FastDetector::FAST12);

    EXPECT_TRUE(abs(fastPtr -> getAdjacentAreaRadius() - 35) < 0.001);
    fastPtr -> setAdjacentAreaRadius(25);
    EXPECT_TRUE(abs(fastPtr -> getAdjacentAreaRadius() - 25) < 0.001);

    EXPECT_EQ(fastPtr -> getOctave(), 1);
    fastPtr -> setOctave(0);
    EXPECT_EQ(fastPtr -> getOctave(), 0);

    EXPECT_FALSE(fastPtr -> gridUsed());
    EXPECT_FALSE(fastPtr -> nmsUsed());

    EXPECT_EQ(fastPtr -> getNumKeypoints(), 0);
    //EXPECT_EQ(fastPtr -> getClassName(), "FastDetector");

    std::vector<sf::Keypoint> keypoints1;
    fastPtr -> detectKeypoints(img1, 5, 5, img1.rows - 10, img1.cols - 10, keypoints1);
    EXPECT_EQ(keypoints1.size(), 0);

    std::vector<sf::Keypoint> keypoints2;
    fastPtr -> detectKeypoints(img2, 5, 5, img2.rows - 10, img2.cols - 10, keypoints2);
    EXPECT_EQ(keypoints2.size(), 0);

/*
    std::vector<sf::Keypoint> keypoints3;
    fastPtr -> detectKeypoints(img3, 0, 0, img3.rows, img3.cols, keypoints3);
    EXPECT_TRUE(keypoints3.size() > 0);
    for(size_t ki = 0, kend = keypoints3.size(); ki<kend; ++ki)
        std::cout << keypoints3[ki] << std::endl;
*/

    std::vector<sf::Keypoint> keypoints4;
    fastPtr -> detectKeypoints(img4, 5, 5, img4.rows - 10, img4.cols - 10, keypoints4);
    EXPECT_TRUE(keypoints4.size() > 0);
    EXPECT_TRUE(fastPtr -> getNumKeypoints() > 0);
    for(size_t ki = 0, kend = keypoints4.size(); ki<kend; ++ki)
        std::cout << keypoints4[ki] << std::endl;
    
    std::vector<sf::Keypoint> keypoints5;
    fastPtr -> detectKeypoints(img5, 5, 5, img5.rows - 10, img5.cols - 10, keypoints5);
    EXPECT_TRUE(keypoints5.size() > 0);
    EXPECT_TRUE(fastPtr -> getNumKeypoints() > 0);
    for(size_t ki = 0, kend = keypoints5.size(); ki<kend; ++ki)
        std::cout << keypoints5[ki] << std::endl;

    //nms test
    fastPtr -> setNms(3);
    EXPECT_TRUE(fastPtr -> nmsUsed());

    std::vector<sf::Keypoint> keypoints6;
    fastPtr -> detectKeypoints(img4, 5, 5, img4.rows - 10, img4.cols - 10, keypoints6);
    EXPECT_TRUE(keypoints6.size() > 0);
    EXPECT_TRUE(fastPtr -> getNumKeypoints() > 0);
    for(size_t ki = 0, kend = keypoints6.size(); ki<kend; ++ki)
        std::cout << keypoints6[ki] << std::endl;


    //grid test
    fastPtr -> gridAssignment(10, 10);
    EXPECT_TRUE(fastPtr -> gridUsed());

    std::vector<sf::Keypoint> keypoints7;
    fastPtr -> detectKeypoints(img4, 5, 5, img4.rows - 10, img4.cols - 10, keypoints7);
    EXPECT_TRUE(keypoints7.size() > 0);
    EXPECT_TRUE(fastPtr -> getNumKeypoints() > 0);
    for(size_t ki = 0, kend = keypoints7.size(); ki<kend; ++ki)
        std::cout << keypoints7[ki] << std::endl;
    
}


}  //namespace