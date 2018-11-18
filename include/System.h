#ifndef DEVO_SYSTEM_H_
#define DEVO_SYSTEM_H_

#include "Global.h"
#include <boost/noncopyable.hpp>

namespace devo {

namespace IOWrap {
class PangolinViewer;
}

class Tracker;
class Optimizer;

class System
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    System(const std::string& strSettingsFile);
    ~System();

    void addImage(const cv::Mat& img, const cv::Mat& img2, double timestamp);
    // void addImage_surf(const cv::Mat& img, const cv::Mat& img2, double timestamp);
    void addImage_aruco(const cv::Mat& img, const cv::Mat& img2, double timestamp);

    void addImage_aruco(const cv::Mat& img, SE3 &vicon_pose, double timestamp);

    void addImage_track(const cv::Mat& img, double timestamp);

    bool determineColor(cv::Mat img, int num);

    int _minFrames;
    int _maxFrames;

    FramePtr _newFrame;

    // Tracking states
    enum State
    {
        SYSTEM_NOT_READY = -1,
        NO_IMAGES_YET = 0,
        NOT_INITIALIZED = 1,
        OK = 2,
        LOST = 3
    };

    State _state;
    State _lastProcessedState;

    cv::Vec3b _ballColor_mean;
    float _ballColor_stdv;

    Tracker* _tracker;
    Optimizer* _optimizer;

  private:
    IOWrap::PangolinViewer* _pangolinViewer;

    bool initialization_aruco(FramePtr newFrame);
};

} // namespace devo

#endif
