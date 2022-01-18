#ifndef TREASURE_OF_SLAM_MAP_MAP_H_
#define TREASURE_OF_SLAM_MAP_MAP_H_

#include <cstdint>
#include <map>

namespace tos{
    class BaseMap(){
        protected:
        typedef uint32_t mapPointId;
        std::map<mapPointId,Point3dSurrogate&> mapPoints;

        public:
        /*-------- construct && destruct --------*/
        virtual BaseMap()=0;
        virtual ~BaseMap()=0;

        /*-------- operator --------*/
        //地图的可视化；
        virtual bool visualize()=0;
    };

    class KeyframeMap:public BaseMap{  //TOTHINK 是否还会有子类继承？
        protected:
        typedef uint32_t keyframeId;
        std::map<keyframeId,Keyframe&> keyframes;  //TODO

        keyframeId maxKeyframeId=0;

        //minimun spanning tree;
        std::map<keyframeId,Keyframe&> mst;

        protected:
        void updateMST(Keyframe& kf);

        public:
        /*-------- construct && destruct --------*/
        KeyframeMap();
        KeyframeMap(const KeyframeMap&)=delete;
        KeyframeMap& operator=(const KeyframeMap&)=delete;
        ~KeyframeMap();

        /*-------- operator --------*/
        addKeyframe(Keyframe& kf);
        deleteKeyframe(Keyframe& kf);
    };
}  //namespace tos;

#endif  //#ifndef;