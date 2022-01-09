//Keypoint.h
#ifndef SLAM_FACTORY_FEATURE_KEYPOINT_H
#define SLAM_FACTORY_FEATURE_KEYPOINT_H

#include <iostream>

#include "Def.h"

namespace sf{
    
/** @brief Keypoint struct;
*** @param x x coordinate of Keypoint;
*** @param y y coordinate of Keypoint;
*** @param adjacentAreaRadius Radius of keypoint adjacent area;
*** @param octave Pyramid octave which the keypoint detected;
*** @param orientation Orientation of keypoint;
*** @param score Output of keypoint detection algorithm;
*** @param classId class of keypoint;
**/
struct SF_EXPORTS Keypoint{

    float x;
    float y;
    float adjacentAreaRadius;
    uint8_t octave;
    double orientation;
    double score;
    uint32_t classId;

    Keypoint() : x(0.0), y(0.0), adjacentAreaRadius(0.0), octave(0), orientation(0.0),  \
                score(0.0), classId(0){}

    Keypoint(float _x, float _y, float _adjacentAreaRadius, uint8_t _octave,  \
                double _orientation, double _score, uint32_t _classId)  \
                : x(_x), y(_y), adjacentAreaRadius(_adjacentAreaRadius), octave(_octave),  \
                orientation(_orientation), score(_score), classId(_classId){}

};

//using namespace std;

#if(1)

std::ostream& operator<<(std::ostream& os, const Keypoint& kp){

    os << kp.x << " " << kp.y << " " << kp.adjacentAreaRadius << " " << kp.octave << " "  \
        << kp.orientation << " " << kp.score << " " << kp.classId;
    
    return os;
}

#endif

}  //namespace

#endif