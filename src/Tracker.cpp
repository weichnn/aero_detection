
#include "Tracker.h"
#include "System.h"
#include "CameraModel.h"

namespace devo {

Tracker::Tracker()
{
    _unfound_count = 0;
}

Tracker::~Tracker()
{
}

bool Tracker::findCircleCenter_Color(cv::Mat image_input, Vec3 &result_return)
{
    int th = 10;
    int num = 0;
    cv::Mat image;
    cv::undistort(image_input, image, CAM_GLOBAL[0]->_cvK, CAM_GLOBAL[0]->_distCoef, CAM_GLOBAL[0]->_cvDK);


    cv::Mat result; // try hsv
    cv::inRange(image, cv::Scalar(255-th,255-th,255-th), cv::Scalar(255,255,255),result);

    Vec3 center_result;

    // imshow("result"+std::to_string(num),result);
    // std::cout << "result"+std::to_string(num) << std::endl;
    // cvWaitKey(1);

    vector<vector<cv::Point> > contours;
    cv::findContours(result, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    cv::RNG rng = cv::RNG(12345);
    if(true)
    {
        vector<cv::Vec4i> hierarchy;

        // Draw contours
        cv::Mat drawing = cv::Mat::zeros( result.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
        {
            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
        }

         // Get the moments
        vector<cv::Moments> mu(contours.size() );
        for( int i = 0; i < contours.size(); i++ )
        { 
            mu[i] = moments( contours[i], false ); 
        }

        ///  Get the mass centers:
        float max_area = 0;
        for( int i = 0; i < contours.size(); i++ )
        { 
            float area = contourArea(contours[i],false);
            if (area > max_area )
            {
                max_area = area;
                cv::Moments mu = moments( contours[i], false );
                // circle( drawing, mc[i], 3, cv::Scalar(0,255,0), -1, 8, 0 );
                center_result = Vec3((mu.m10/mu.m00 - CAM_GLOBAL[0]->_width/2.0)/CAM_GLOBAL[0]->_fx, (mu.m01/mu.m00 - CAM_GLOBAL[0]->_height/2.0)/CAM_GLOBAL[0]->_fx, 1);
                float d = 0.08*CAM_GLOBAL[0]->_fx/sqrt(max_area);
                result_return = center_result*d;
            }

        }
        // imshow("drawing",drawing);
 
        if(contours.size()!=1)
        {
            _unfound_count++;
            return false;
        } 
        // std::cout << max_area << std::endl;
        if(max_area<100.0f)
        {
            return false;
        }
        // cvWaitKey(0);
        return true;
        
    }
}

void Tracker::triangulate(const Vec2 &kp1, const Vec2 &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1[0]*P1.row(2)-P1.row(0);
    A.row(1) = kp1[1]*P1.row(2)-P1.row(1);
    A.row(2) = kp2[0]*P2.row(2)-P2.row(0);
    A.row(3) = kp2[1]*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}


} // end of namespace