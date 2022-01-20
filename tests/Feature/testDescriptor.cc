//testDescriptor.cc

#include "Descriptor.h"
#include "Detector.h"
#include "Image.h"
#include "Keypoint.h"
#include "Types.h"

#include "gtest/gtest.h"

#include <bitset>
#include <cmath>
#include <iostream>
#include <vector>

namespace{

class TestDescriptor : public testing::Test{

protected:
    
    void SetUp() SF_OVERRIDE{
        
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


        sf::MatXui8 mat1 = sf::MatXui8::Zero(300, 300);
        mat1.block(5, 5, 20, 20) = corner3;  
        mat1.block(200, 200, 20, 20) = corner3; 
        mat1.block(100, 50, 20, 20) = corner2; 
        img1 = sf::Image(mat1);  
    }

    sf::Image img1;
};

TEST_F(TestDescriptor, BriefDescriptorBase){

    sf::SharedPtr<sf::BriefDescriptor> briefPtr  \
            = sf::BriefDescriptor::createDescriptorPtr(2, 256,  \
            15, sf::BriefDescriptor::NORMAL);
    
    EXPECT_EQ(briefPtr -> getAdjacentAreaRadius(), 15);
    EXPECT_EQ(briefPtr -> getNumComparedPoints(), 2);
    EXPECT_EQ(briefPtr -> getDescriptionLength(), 256);
    EXPECT_EQ(briefPtr -> getRandomType(), sf::BriefDescriptor::NORMAL);
    
    briefPtr -> setAdjacentAreaRadius(16);
    //briefPtr -> setNumComparedPoints(3);  //ok
    briefPtr -> setDescriptionLength(128);
    briefPtr -> setRandomType(sf::BriefDescriptor::UNIFORM);

    EXPECT_EQ(briefPtr -> getAdjacentAreaRadius(), 16);
    //EXPECT_EQ(briefPtr -> getNumComparedPoints(), 3);  //ok
    EXPECT_EQ(briefPtr -> getDescriptionLength(), 128);
    EXPECT_EQ(briefPtr -> getRandomType(), sf::BriefDescriptor::UNIFORM);
}


static void printDescriptions(const std::vector<std::vector<void*>>& descriptions, 
        sf::SharedPtr<sf::BriefDescriptor> briefPtr){
    
    std::bitset<8> bin;

    for(size_t kki = 0, kkend = descriptions.size(); kki < kkend; ++kki){
        for(size_t ki = 0, kend = descriptions[kki].size(); ki < kend; ++ki){

            uint8_t* desPtr = static_cast<uint8_t*>(descriptions[kki][ki]);
            for(size_t k = 0; k < briefPtr -> getDescriptionLength()/sf::BriefDescriptor::BRIEF_BIT_EACH_INT; ++k){

                bin = *(desPtr+k);
                std::cout << bin;
            }
            
            std::cout<<std::endl;
        }
    }
}


TEST_F(TestDescriptor, BriefDescriptorCompute){
    
    sf::SharedPtr<sf::FastDetector> fastPtr = sf::FastDetector::createDetectorPtr(100, 30, 8, 15, 1);

    sf::SharedPtr<sf::BriefDescriptor> briefPtr  \
            = sf::BriefDescriptor::createDescriptorPtr(2, 256,  \
            15, sf::BriefDescriptor::NORMAL);

    std::vector<std::vector<sf::Keypoint>> keypoints1;
    fastPtr -> detectKeypoints(img1, 5, 5, img1.rows - 10, img1.cols - 10, keypoints1);

    std::vector<std::vector<void*>> descriptions1;
    descriptions1.resize(keypoints1.size());
    for(size_t i = 0, iend = descriptions1.size(); i < iend; ++i){
        
        descriptions1[i].resize(keypoints1[i].size());
        for(size_t j = 0, jend =descriptions1[i].size(); j < jend; ++j){

            descriptions1[i][j] = new uint8_t[32];
        }
    }

    briefPtr -> computeDescription(img1, keypoints1, descriptions1);

    printDescriptions(descriptions1, briefPtr);

    for(size_t i = 0, iend = descriptions1.size(); i < iend; ++i){
        
        for(size_t j = 0, jend =descriptions1[i].size(); j < jend; ++j){

            uint8_t* desPtr = static_cast<uint8_t*>(descriptions1[i][j]);
            
            delete [] desPtr;
        }
    }


}


}  //namespace