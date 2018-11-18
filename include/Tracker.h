#ifndef DEVO_TRACKER_H_
#define DEVO_TRACKER_H_

#include "Global.h"
#include <boost/noncopyable.hpp>

namespace devo {

class Tracker
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<Vec2> _centers;

    int _unfound_count;

    Tracker();
    ~Tracker();

    bool trackRobotic(std::vector<cv::Mat> frames, cv::Mat &result);
    bool trackRobotic_color(std::vector<cv::Mat> frames, cv::Vec3b object_color, float stdv_color, cv::Mat &result);
    void triangulate(const Vec2 &kp1, const Vec2 &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);
    bool findCircleCenter_Color(cv::Mat image, Vec2 &result, cv::Vec3b color, float th, int num);

    bool findCircleCenter_Color(cv::Mat image, Vec3 &result);

}; // end of class

} // namespace devo

#endif
