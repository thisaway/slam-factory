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
    int adjacentAreaRadius;
    int octave;
    double orientation;
    double score;
    int classId;

    Keypoint() : x(0.0), y(0.0), adjacentAreaRadius(0), octave(0), orientation(0.0),  \
                score(0.0), classId(0){}

    Keypoint(float _x, float _y, int _adjacentAreaRadius, int _octave,  \
                double _orientation, double _score, int _classId)  \
                : x(_x), y(_y), adjacentAreaRadius(_adjacentAreaRadius), octave(_octave),  \
                orientation(_orientation), score(_score), classId(_classId){}

};

//using namespace std;

#if(1)

std::ostream& operator<<(std::ostream& os, const Keypoint& kp);

#endif

}  //namespace

#endif