
#include "Frame.h"
#include "CameraModel.h"


namespace devo {

long unsigned int Frame::_frame_counter = 0;
long unsigned int Frame::_KF_counter = 0;


Frame::Frame(const cv::Mat& img, CameraModel* camModel, double timestamp)
  : _id(_frame_counter++)
  , _timestamp(timestamp)
  , _is_keyframe(false)
  , _debugMode(false)
  , _hasMarker(false)
{
    _camID = camModel->_camID;
    _kf_id = 0;

    if(img.channels()==3)
    {
        {
            _image_raw = img.clone();
            cvtColor(_image_raw,_image,CV_RGB2GRAY);
        }

    }
    else
    {
        _image_raw = img.clone();
        _image = img.clone();
    }

    _hasMarker = init();
}

Frame::~Frame()
{

}

bool
Frame::init()
{
    bool result = true;
    // Try to initialize
    int count = 0;
    // for(auto it:newFrame)
    {
        cv::Mat srcImage = _image_raw.clone();

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
            for(int i = 0; i <markerIds.size(); i++)
            {
                
                cv::aruco::drawAxis(srcImage, CAM_GLOBAL[count]->_cvK, CAM_GLOBAL[count]->_distCoef, rvecs[i], tvecs[i], 0.2);
                cv::Mat R;
                cv::Mat t(3,1,CV_64F);
                t.at<double>(0, 0) = tvecs[i][0];
                t.at<double>(1, 0) = tvecs[i][1];
                t.at<double>(2, 0) = tvecs[i][2];
                Rodrigues(rvecs[i], R);
                // CAM_GLOBAL[count]->_R = R.clone();
                // CAM_GLOBAL[count]->_t = t.clone(); 
                CAM_GLOBAL[count]->setPose(R, t);

                Eigen::Matrix<double, 3, 3> M;
                M << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), 
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), 
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
                Vec3 eigent(t.at<double>(0, 0),t.at<double>(1, 0),t.at<double>(2, 0));
                _T_f_w = (SE3(M,eigent).inverse());

                cout << _T_f_w.inverse().matrix() << endl;
                cout << "marker " << markerIds[i] << ": " << t << endl;

            }
            imshow("out"+std::to_string(count), srcImage);
            cvWaitKey(1);
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



} // end of name space
