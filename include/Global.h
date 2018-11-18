

#ifndef DEVO_GLOBAL_H_
#define DEVO_GLOBAL_H_

#include "NumType.h"
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <list>
#include <math.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <vector>
#include <opencv2/aruco.hpp>

#include <sys/time.h>

// #include "opencv2/nonfree/features2d.hpp"


#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

namespace devo {

class CameraModel;
class Frame;

typedef Frame* FramePtr;

extern std::vector<CameraModel*> CAM_GLOBAL;
extern int CAMNUM;

} // namespace devo

#endif // devo_GLOBAL_H_
