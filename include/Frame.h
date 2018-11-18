

#ifndef DEVO_FRAME_H_
#define DEVO_FRAME_H_

#include "Global.h"
#include <boost/noncopyable.hpp>

namespace g2o {
class VertexSE3Expmap;
}
typedef g2o::VertexSE3Expmap g2oFrameSE3;

namespace devo {

class CameraModel;

class Frame //: public std::enable_shared_from_this<Frame>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static long unsigned int _frame_counter; //!< Counts the number of created frames. Used to set the unique id.
    static long unsigned int _KF_counter;    //!< Counts the number of created frames. Used to set the unique id.

    int _camID;

    long unsigned int _id;    //!< Unique id of the frame.
    long unsigned int _kf_id; //!< Unique id of the frame.
    double _timestamp;        //!< Timestamp of when the image was recorded.
    SE3 _T_f_w;               //!< Transform (f)rame from (w)orld.
    SE3 _T_f_w_GBA;           // after ba
    SE3 _T_c_p;
    Mat66 _Cov;        //!< Covariance.

    bool _is_keyframe; //!< Was this frames selected as keyframe?
    cv::Mat _image_raw;
    cv::Mat _image;
    cv::Mat _depth;

    bool _debugMode;

    bool _hasMarker;

    Frame(const cv::Mat& img, CameraModel* camModel, double timestamp);
    ~Frame();

  private:
    bool init();

};
}

#endif
