//sensor.h
#ifndef TREASURE_OF_SLAM_SENSOR_SENSOR_H_
#define TREASURE_OF_SLAM_SENSOR_SENSOR_H_

#include <cstdint>
#include <queue>

#ifndef TREASURE_OF_SLAM__CORE_POSE_H_
#include "core/pose.h"
#endif

#ifndef TREASURE_OF_SLAM_SENSOR_SENSORDATA_H_
#include "sensor/sensor_data.h"
#endif

namespace tos{
    //纯虚类
    //Sensor类用于仿真现实世界中的传感器；
    //设置有缓冲区，用于解决收发数据频率可能不一致的问题，同时考虑到潜在的多线程需求；
    class Sensor{  
        protected:
        uint64_t timeStamp_;  //纳秒

        //传感器坐标系到基坐标系；
        Pose& externalReference_;

        //传感器发布数据的频率 /Hz
        unsigned int frequency_;

        //传感器缓冲区,存放矫正后的数据;
        std::queue<SensorDataSurrogate&> buffer_;

        protected:
        //矫正传感器的数据；
        virtual bool rectify()=0;

        public:
        /*-------- set --------*/
        inline void setExternalReference(const Pose& pose){
            externalReference_=pose;
        }

        inline void setFrequency(unsigned int frequency){
            frequency_=frequency;
        }
        
        /*-------- access --------*/
        inline uint64_t getTimeStamp() const{
            return timeStamp_;
        }

        inline Pose& getExternalReference() const{
            return externalReference_;
        }

        inline unsigned int getFrequency() const{
            return frequency_;
        }

        /*--------- operator --------*/
        //从传感器缓冲区读取一个数据；
        //RETURN:读取成功返回1，读取失败返回0；
        SensorDataSurrogate read();

        //接收传感器数据的代理，进行数据校正的操作后，送入传感器的缓冲区；
        bool receive(SensorDataSurrogate&);
    };
}  //namespace tos;

#endif  //#ifndef