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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"

using namespace cv;
using namespace std;

devo::System* _system;

void GrabSyncData(const sensor_msgs::ImageConstPtr& msgRGB,const geometry_msgs::PoseStamped& msgV)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    {
        devo::Vec3 posi = devo::Vec3(msgV.pose.position.x,msgV.pose.position.y,msgV.pose.position.z);
        Eigen::Quaternion<double> quat(msgV.pose.orientation.w, msgV.pose.orientation.x,msgV.pose.orientation.y,msgV.pose.orientation.z);
        devo::SE3 vicon_pose = devo::SE3(quat,posi);

        _system->addImage_aruco(cv_ptr->image, vicon_pose, cv_ptr->header.stamp.toSec());
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

    ros::init(argc, argv, "calibator");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> vicon_sub(nh, "/mavros/vision_pose/pose", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,vicon_sub);
    sync.registerCallback(GrabSyncData);

    ros::spin();

    delete _system;

    return 0;
}