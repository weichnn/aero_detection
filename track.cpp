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

using namespace cv;
using namespace std;


 
int main(int argc, char** argv)
{


    std::cout << "start!" << std::endl;

    if (argc != 2) {
        cerr << endl << "not enough para input" << endl;
        return 1;
    }

    devo::System* _system = new devo::System(argv[1]);

    VideoCapture cap0(0); // open the default camera0

    if(!cap0.isOpened())  // check if we succeeded
        return -1;

    long count = 0;
    bool determinedColor = false;
    for(;;)
    {

        Mat frame0;
        cap0 >> frame0; // get a new frame from camera


        //         struct timeval tv_start;
        // gettimeofday(&tv_start, NULL);

        if(frame0.empty())
        {
            printf("error occurs.\n");
            continue;
        }
        // if(!determinedColor)
        // {
        //     determinedColor = _system->determineColor(frame0,count);
        //     if(determinedColor)
        //     {
        //         cap0 >> frame0; // get a new frame from camera
        //         cap1 >> frame1; // get a new frame from camera
        //         sleep(5);
        //         cap0 >> frame0; // get a new frame from camera
        //         cap1 >> frame1; // get a new frame from camera
        //     }
        // }
        else
        {
            // imshow("test",frame0);
            // imwrite("image.png",frame0);
            // cvWaitKey(33);
            _system->addImage_track(frame0, count);
        }

        // struct timeval tv_end;
        // gettimeofday(&tv_end, NULL);
        // std::cout << "need time(ms):" << ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f) << std::endl;
        count++;
    }

    return 0;
}
