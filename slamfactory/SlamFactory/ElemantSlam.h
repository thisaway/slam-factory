#ifndef ELEMANT_SLAM_LIBRARY_ELEMANTSLAM_ELEMANTSLAM_H
#define ELEMANT_SLAM_LIBRARY_ELEMANTSLAM_ELEMANTSLAM_H

#ifndef ELEMANT_SLAM_LIBRARY_CORE_CORE_H
#include "Core/Core.h"
#endif

namespace esl{
    class ESL_EXPORTS Slam{
        //线程池
    public:
        virtual ~Slam()=0;
        void run()=0;
    };
}

#endif