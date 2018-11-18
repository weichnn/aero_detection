#ifndef DEVO_CAMERA_MODEL_H_
#define DEVO_CAMERA_MODEL_H_

#include <Eigen/Geometry>
#include "Global.h"


namespace devo {

using namespace std;

class CameraModel
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int _camID;

    int _width, _height;

    double _fx, _fy;
    double _invfx, _invfy;
    double _cx, _cy;

    double _factor;

    double _k1, _k2, _k3;

    double _p1, _p2;

    int _fps;

    double _bf;
    double _b;

    cv::Mat _distCoef;
    cv::Mat _cvK;
    cv::Mat _cvDK;

    float _minX;
    float _maxX;
    float _minY;
    float _maxY;

    cv::Mat _R;
    cv::Mat _t;

    cv::Mat _P;

    cv::Mat _Tcw;

    cv::Mat _image_raw;
    cv::Mat _image;

    CameraModel(int camID, double width, double height, double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2, int fps)
      : _width(width)
      , _height(height)
      , _fx(fx)
      , _fy(fy)
      , _cx(cx)
      , _cy(cy)
      , _k1(k1)
      , _k2(k2)
      , _k3(k3)
      , _p1(p1)
      , _p2(p2)
      , _fps(fps)
    {
        if (_fps == 0) {
            _fps = 30;
        }

        _camID = camID;

        _maxX = _width;
        _maxY = _height;

        if (fabs(_factor) < 1e-5)
            _factor = 1;
        else
            _factor = 1.0f / _factor;

        _invfx = 1.0f / _fx;
        _invfy = 1.0f / _fy;

        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = _fx;
        K.at<float>(1,1) = _fy;
        K.at<float>(0,2) = _cx;
        K.at<float>(1,2) = _cy;
        K.copyTo(_cvK);

        K.at<float>(0,0) = _fx;
        K.at<float>(1,1) = _fx;
        K.at<float>(0,2) = _width/2.0;
        K.at<float>(1,2) = _height/2.0;
        K.copyTo(_cvDK);


        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = _k1;
        DistCoef.at<float>(1) = _k2;
        DistCoef.at<float>(2) = _p1;
        DistCoef.at<float>(3) = _p1;
        if (_k3 != 0) {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = _k3;
        }
        DistCoef.copyTo(_distCoef);
    }

    void setImage(cv::Mat image)
    {
        _image_raw = image.clone();
        _image = image.clone();
    }

    void setPose(cv::Mat R, cv::Mat t)
    {
        _R=R.clone();
        _t=t.clone();
        cv::Mat P1 = cv::Mat::zeros(3,4,CV_32F);
        _R.copyTo(P1.rowRange(0,3).colRange(0,3));
        _t.copyTo(P1.rowRange(0,3).col(3));
        _P = (_cvK*P1);
        cv::Mat Pose1 = cv::Mat::eye(4,4,CV_32F);
        P1.copyTo(Pose1.rowRange(0,3).colRange(0,4));
        _Tcw = Pose1;
        
    }

    void ComputeImageBounds(const cv::Mat& imLeft)
    {

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = _fx;
        K.at<float>(1, 1) = _fy;
        K.at<float>(0, 2) = _cx;
        K.at<float>(1, 2) = _cy;
        K.copyTo(_cvK);

        if (_distCoef.at<float>(0) != 0.0) {
            cv::Mat mat(4, 2, CV_32F);
            mat.at<float>(0, 0) = 0.0;
            mat.at<float>(0, 1) = 0.0;
            mat.at<float>(1, 0) = _width;
            mat.at<float>(1, 1) = 0.0;
            mat.at<float>(2, 0) = 0.0;
            mat.at<float>(2, 1) = _height;
            mat.at<float>(3, 0) = _width;
            mat.at<float>(3, 1) = _height;

            // Undistort corners
            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, K, _distCoef, cv::Mat(), K);
            mat = mat.reshape(1);

            _minX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
            _maxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
            _minY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
            _maxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));

        } else {
            _minX = 0.0f;
            _maxX = _width;
            _minY = 0.0f;
            _maxY = _height;
        }

    }

    bool isVisible(const float& u, const float& v)
    {
        if (u < _minX || u > _maxX)
            return false;
        if (v < _minY || v > _maxY)
            return false;
        return true;
    }
};
}

#endif
