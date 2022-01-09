#ifndef SLAM_FACTORY_SENSOR_IMAGE_H
#define SLAM_FACTORY_SENSOR_IMAGE_H

#include "Def.h"
#include "Types.h"

namespace sf{
    class Image{
    public:
        Image();
        Image(uint32_t r,uint32_t c,uint8_t iniValue=0);
        Image(const MatXui8& img);
        Image(const Image& img);
        Image& operator=(const Image& img);

        inline const uint8_t& operator()(const uint32_t& r,const uint32_t& c) const{
            return data(r,c);
        }

        ~Image();
    
    public:
        uint32_t rows;
        uint32_t cols;
    
    protected:
        MatXui8 data;
    };
}  //namespace

#endif