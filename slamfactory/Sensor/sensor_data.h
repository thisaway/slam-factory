//sensor_data.h
#ifndef TREASURE_OF_SLAM_SENSOR_SENSORDATA_H_
#define TREASURE_OF_SLAM_SENSOR_SENSORDATA_H_

#include <cstdint>

namespace tos{
    //SensorData的代理；
    class SensorDataSurrogate{
        private:
        SensorData* sensorDataPtr_;

        public:
        /*-------- construct and destruct ---------*/
        SensorDataSurrogate();
        SensorDataSurrogate(const SensorData& sd);
        ~SensorDataSurrogate();
        SensorDataSurrogate(const SensorDataSurrogate& sds);
        SensorDataSurrogate& operator=(const SensorDataSurrogate& sds);

        //common op
        void setTimeStamp(uint64_t timeStamp);
        uint64_t getTimeStamp() const;
        SensorDataType getType() const;  //may be a mistake;
        friend ostream& operator<<(ostream& os,SensorDataSurrogate& sds);

        //Image op
        uint16_t getRows() const;
        uint16_t getCols() const;
        uint8_t getChannels() const;
        uint16_t operator()(uint16_t rows,uint16_t cols,uint8_t channels) const;

    };
    ostream& operator<<(ostream& os,SensorDataSurrogate& sds);

    class SensorData{
        public:
        static enum SensorDataType{ImageType=0};

        protected:
        //传感器数据类型
        SensorDataType type_;
        uint64_t timeStamp_;  //纳秒

        public:
        /*-------- construct and deconstruct --------*/
        virtual SensorData()=0;
        virtual ~SensorData()=0;

        /*--------- set ----------*/
        inline void setTimeStamp(uint64_t timeStamp){
            timeStamp_=timeStamp;
        }

        /*--------- get ---------*/
        inline uint64_t getTimeStamp() const{
            return timeStamp_;
        }

        inline SensorDataType getType() const{
            return type_;
        }
    };

    class Image final:public SensorData{
        private:
        uint16_t rows_;
        uint16_t cols_;
        uint8_t channels_;

        //动态存储空间，存放图像的数据；
        //rows_*cols_*channel_大小；
        uint16_t* imagePtr_;

        public:
        /*-------- construct and destruct ---------*/
        Image();
        Image(uint16_t rows,uint16_t cols,uint8_t channels);
        Image(const Image& img);
        Image(const Image&& img);
        ~Image();

        /*-------- access --------*/
        inline uint16_t getRows() const{
            return rows_;
        }

        inline uint16_t getCols() const{
            return cols_;
        }

        inline uint8_t getChannels() const{
            return channels_;
        }

        uint16_t operator()(uint16_t rows,uint16_t cols,uint8_t channels) const;

        /*-------- operator ---------*/
        friend ostream& operator<<(ostream& os,Image& img);
    };

    //Image类输出流重载；
    ostream& operator<<(ostream& os,Image& img);
}  //namespace tos;

#endif  //#ifndef