

#include "Tracker.h"
#include "System.h"
#include "CameraModel.h"
#include "Frame.h"
#include "Optimizer.h"

namespace devo {

std::vector<CameraModel*> CAM_GLOBAL;
int CAMNUM;

System::System(const std::string& strSettingsFile)
  : _state(NO_IMAGES_YET)
{
    // Check settings file
    cv::FileStorage fSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    if (!fSettings.isOpened()) {
        std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
        exit(-1);
    }

    float fps = fSettings["Camera.fps"];
    if (fps == 0) {
        fps = 30;
    }

    // Max/Min Frames to insert keyframes and to check relocalisation
    _minFrames = 0;
    _maxFrames = fps;

    CAMNUM = 1;
    CAM_GLOBAL.push_back(
                 new CameraModel((int)fSettings["Camera0.id"],
                                 (double)fSettings["Camera0.width"],
                                 (double)fSettings["Camera0.height"],
                                 (double)fSettings["Camera0.fx"],
                                 (double)fSettings["Camera0.fy"],
                                 (double)fSettings["Camera0.cx"],
                                 (double)fSettings["Camera0.cy"],
                                 (double)fSettings["Camera0.k1"],
                                 (double)fSettings["Camera0.k2"],
                                 (double)fSettings["Camera0.k3"],
                                 (double)fSettings["Camera0.p1"],
                                 (double)fSettings["Camera0.p2"],
                                 (int)fSettings["Camera0.fps"]
                                 )
    );


    _tracker = new Tracker();
    _optimizer = new Optimizer();

}

System::~System()
{
    delete _optimizer;
    for(auto it:CAM_GLOBAL)
    {
        delete it;
    }

}


bool System::determineColor(cv::Mat img, int num)
{
    cv::Mat mean;
    cv::Mat stdv;
    cv::meanStdDev(img,mean,stdv);
    if(cv::norm(stdv) < 20)
    {
        _ballColor_stdv = cv::norm(stdv);
        _ballColor_mean = mean;
        std::cout << mean << stdv << std::endl;
        return true;
    }
    return false;
}

void
System::addImage_track(const cv::Mat& img0, double timestamp)
{
    Vec3 result;
    if(_tracker->findCircleCenter_Color(img0, result))
    {
        std::cout << result.transpose() << std::endl;
    }
}

void
System::addImage_aruco(const cv::Mat& img0, SE3 &vicon_pose, double timestamp)
{

    _newFrame = new Frame(img0, CAM_GLOBAL[0], timestamp);

    if (_state == NO_IMAGES_YET) {
        _state = NOT_INITIALIZED;
    }
    _lastProcessedState = _state;

    if (_newFrame->_hasMarker)
    {
        // SE3 temp = SE3(Mat33::Identity(),Vec3(1,0,0));
        // SE3 t2 = temp * _newFrame->_T_f_w * temp.inverse();
        // std::cout << temp.matrix() << std::endl;

        cout << "The image frame pose:" <<  _T_f_w.inverse().matrix() << endl;
        cout << "The image frame pose:" <<  vicon_pose.matrix() << endl;
        
        _optimizer->optimizeLoop(_newFrame->_T_f_w, vicon_pose);
    }
    
    delete _newFrame;

    return;
}

bool
System::initialization_aruco(FramePtr it)
{
    bool result = true;
    // Try to initialize
    int count = 0;
    // for(auto it:newFrame)
    {
        cv::Mat srcImage = it->_image_raw.clone();

        cv::Mat bi;
        //adaptiveThreshold(gray, bi, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 33, 0 );
        //threshold(gray, bi, 12, 255, CV_THRESH_BINARY);

        //cv::GaussianBlur(srcImage, dstImage, Size(3,3), 0, 0);
        //cv::bilateralFilter(srcImage,dstImage,0,10,10);
        cv::Mat dstImage;
        cv::medianBlur(srcImage, dstImage, 3);
        std::vector<int> markerIds;
        std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        //cout << "zyx: " << parameters->maxMarkerPerimeterRate << "  " << parameters->adaptiveThreshConstant << endl;
        parameters->adaptiveThreshWinSizeMin = 13;
        parameters->adaptiveThreshWinSizeMax = 33;
        parameters->adaptiveThreshWinSizeStep = 10;
        parameters->adaptiveThreshConstant = 7;
        parameters->minMarkerPerimeterRate = 0.06;
        parameters->polygonalApproxAccuracyRate = 0.06;
        //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        
        // cv::adaptiveThreshold(dstImage, bi, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 13, parameters->adaptiveThreshConstant );

        cv::aruco::detectMarkers(srcImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        //if(rejectedCandidates.size() > 0)
        //    drawContours(srcImage, rejectedCandidates, -1, Scalar(255,0,0));
        if(markerIds.size()>0)
        {
        
            cv::aruco::drawDetectedMarkers(srcImage, markerCorners, markerIds);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.2, CAM_GLOBAL[count]->_cvK, cv::noArray(), rvecs, tvecs);
            cv::aruco::drawDetectedMarkers(srcImage, markerCorners, markerIds);
            for(int i = 0; i <markerIds.size(); i++)
            {
                
                cv::aruco::drawAxis(srcImage, CAM_GLOBAL[count]->_cvK, CAM_GLOBAL[count]->_distCoef, rvecs[i], tvecs[i], 7);
                cv::Mat R;
                cv::Mat t(3,1,CV_64F);
                t.at<double>(0, 0) = tvecs[i][0];
                t.at<double>(1, 0) = tvecs[i][1];
                t.at<double>(2, 0) = tvecs[i][2];
                Rodrigues(rvecs[i], R);
                // CAM_GLOBAL[count]->_R = R.clone();
                // CAM_GLOBAL[count]->_t = t.clone(); 
                CAM_GLOBAL[count]->setPose(R, t);
                cout << "camera center: " << -R.inv() * t << endl;
                cout << "marker " << markerIds[i] << ": " << t << endl;

            }
            // imshow("out"+std::to_string(count), srcImage);
            // cvWaitKey(1);
        }
        else
        {
            result = false;
            std::cout << "there is no marker" << std::endl;
        }

        markerIds.clear();
        markerCorners.clear();
        count++;
    }
    return result;
}


}
