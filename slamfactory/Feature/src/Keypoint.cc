//Keypoint.cc
#include "Keypoint.h"

namespace sf{

std::ostream& operator<<(std::ostream& os, const Keypoint& kp){

    os << kp.x << " " << kp.y << " " << kp.adjacentAreaRadius << " " << kp.octave << " "  \
        << kp.orientation << " " << kp.score << " " << kp.classId;
    
    return os;
}

}  //namespace