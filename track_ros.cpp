#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <string.h>
#include "stdio.h"
#include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/nonfree/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "System.h"
#include <opencv2/opencv.hpp>

#include "Global.h"

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>


using namespace cv;
using namespace std;

devo::System* _system;

void GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    {
        _system->addImage_track(cv_ptr->image,cv_ptr->header.stamp.toSec());
    }
}

 
int main(int argc, char** argv)
{


    std::cout << "start!" << std::endl;

    if (argc != 3) {
        cerr << endl << "not enough para input" << endl;
        // return 1;
    }

    ;
    _system = new devo::System("src/aero_detect/config/config.yaml");

    ros::init(argc, argv, "tracker");

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/cv_camera/image_raw", 1, GrabImage);

    ros::spin();

    delete _system;

    return 0;
}
