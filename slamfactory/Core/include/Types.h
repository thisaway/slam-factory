#ifndef SLAM_FACTORY_CORE_TYPES_H
#define SLAM_FACTORY_CORE_TYPES_H

#ifndef SLAM_FACTORY_CORE_SFDEF_H
#include "Def.h"
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>

namespace sf{

/** @brief Box(_0, _1, _2, _3) indicates box position；
*** @param (_0, _1) is the position of the upper left corner of the box;
*** @param (_2, _3) are the rows and columns of the box;
**/
typedef Eigen::Vector4i                  Box;

typedef Eigen::Quaternion<double>        Quaternion;
typedef Eigen::Matrix<double, 3, 1>      Position3d;
typedef Eigen::Matrix<double, 3, 1>      Angles;
typedef Eigen::Matrix<double, 3, 3>      Rotation;
typedef Eigen::Matrix<double, 4, 4>      Transformation;


template <typename _Tp>
struct SF_EXPORTS Point2D{

    _Tp x;
    _Tp y;
    Point2D(_Tp _x, _Tp _y) : x(_x), y(_y) {}
    Point2D() : x(0), y(0){}
};

typedef Point2D<double>         Point2d;
typedef Point2D<float>          Point2f;
typedef Point2D<int>            Point2i;

typedef Point2D<int8_t>         Point2i8;
typedef Point2D<int16_t>        Point2i16;
typedef Point2D<int32_t>        Point2i32;
typedef Point2D<int64_t>        Point2i64;

typedef Point2D<uint8_t>        Point2ui8;
typedef Point2D<uint16_t>       Point2ui16;
typedef Point2D<uint32_t>       Point2ui32;
typedef Point2D<uint64_t>       Point2ui64;


template<typename _Tp>
struct SF_EXPORTS Point3D{

    _Tp x;
    _Tp y;
    _Tp z;
    Point3D(_Tp _x, _Tp _y, _Tp _z) : x(_x), y(_y), z(_z){}
    Point3D() : x(0), y(0), z(0){}
};

typedef Point3D<double>        Point3d;
typedef Point3D<float>         Point3f;
typedef Point3D<int>           Point3i;

typedef Point3D<int8_t>        Point3i8;
typedef Point3D<int16_t>       Point3i16;
typedef Point3D<int32_t>       Point3i32;
typedef Point3D<int64_t>       Point3i64;

typedef Point3D<uint8_t>       Point3ui8;
typedef Point3D<uint16_t>      Point3ui16;
typedef Point3D<uint32_t>      Point3ui32;
typedef Point3D<uint64_t>      Point3ui64;

template <typename _Tp>
using SharedPtr = std::shared_ptr<_Tp>;


template <typename _Scalar, int _Rows, int _Cols> 
using Mat = Eigen::Matrix<_Scalar, _Rows, _Cols>;

const int Dynamic = -1;
using MatXd = Eigen::Matrix<double, Dynamic, Dynamic>;
using MatXf = Eigen::Matrix<float, Dynamic, Dynamic>;
using MatXi = Eigen::Matrix<int, Dynamic, Dynamic>;

using MatXi8 = Eigen::Matrix<int8_t, Dynamic, Dynamic>;
using MatXi16 = Eigen::Matrix<int16_t, Dynamic, Dynamic>;
using MatXi32 = Eigen::Matrix<int32_t, Dynamic, Dynamic>;
using MatXi64 = Eigen::Matrix<int64_t, Dynamic, Dynamic>;

using MatXui8 = Eigen::Matrix<uint8_t, Dynamic, Dynamic>;
using MatXui16 = Eigen::Matrix<uint16_t, Dynamic, Dynamic>;
using MatXui32 = Eigen::Matrix<uint32_t, Dynamic, Dynamic>;
using MatXui64 = Eigen::Matrix<uint64_t, Dynamic, Dynamic>;


//Description
template <typename _Tp, int _Length> 
using Description = Eigen::Matrix<_Tp, 1, _Length>;

typedef uint8_t BriefMetaType;
typedef Description<BriefMetaType, 32> BriefDescription256;
typedef Description<BriefMetaType, 16> BriefDescription128;
typedef Description<BriefMetaType, 8>  BriefDescription64;


/*
class SF_EXPORTS Pose{
private:
    Quaternion posture;
    Position3d position;
public:
    Pose();
    Pose(const SF_IN Quaterniond& q,const SF_IN Position3d& p):posture(q),position(p){}
    Pose(SF_IN Quaterniond& q,SF_IN Position3d& p);

    /** @brief Euler angles construct Pose;
    *** @param a Angles(_1,_2,_3), and (_3,_2,_1) is the order of rotation;
    **/
    /*
    Pose(const SF_IN Angles& a,const SF_IN Position3d& p);

    /** @brief Rotation matrix construct Pose;
    *** @param r Rotation matrix;
    **/
    /*
    Pose(const SF_IN Rotation& r,const SF_IN Position3d& p);
    
    /** @brief Transformation matrix construct Pose;
    *** @param t Transformation matrix;
    **/
    /*
    Pose(const SF_IN Transformation& t);

    Pose(const Pose& p);
    Pose(const Pose&& p);
    Pose& operator=(const Pose& p);
    ~Pose();

    void setPosition(const SF_IN Position3d& p);
    void setPosition(SF_IN Position3d& p);
    SF_OUT const Position3d& getPosition() const;
    SF_OUT Position3d& getPosition() const;

    void setPosture(const SF_IN Quaternion& q);
    void setPosture(SF_IN Quaternion& q);
    void setPosture(const SF_IN Rotation& r);
    void setPosture(const SF_IN Angles& a);
    SF_OUT const Quaternion& getPosture() const;
    SF_OUT Quaternion& getPosture() const;

    SF_OUT Rotation getRotation() const;
    SF_OUT Transformation getTransformation() const;

    //TODO
    /** @brief Get euler angles from Pose;
    *** @param (_0,_1,_2) parameter's range is 0-2, 0 means roll angle,1 means pitch angle,  \
    *** 2 means yaw angle. And (_0,_1,_2) is the order of rotation. For example (2,1,0)  \
    *** means yaw is first rotation,then pitch,finally roll.
    *** @return Angles(_0,_1,_2). _0 is yaw, _1 is pitch, _2 is roll.
    **/
    /*
    SF_OUT Angles getAngles(SF_IN uint8_t range1,SF_IN uint8_t range2,SF_IN uint8_t range3);

    friend SF_OUT ostream& operator<<(SF_IN ostream& os,const SF_IN Pose& p);

};

SF_OUT ostream& operator<<(SF_IN ostream& os,const SF_IN Pose& p);
*/

}

#endif