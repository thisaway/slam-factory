#ifndef SLAM_FACTORY_SENSOR_IMAGE_H
#define SLAM_FACTORY_SENSOR_IMAGE_H

#include "Def.h"
#include "Types.h"

namespace sf{

class Image{

public:

    Image();
    Image(int r,int c, uint8_t iniValue=0);
    Image(const MatXui8& img);
    Image(const Image& img);
    Image& operator=(const Image& img);

    inline const uint8_t& operator()(const int& r, const int& c) const{
        return data(r,c);
    }

    ~Image();

public:

    int rows;
    int cols;

protected:

    MatXui8 data;
};

}  //namespace

#endif