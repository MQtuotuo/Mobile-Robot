
#include <ros/ros.h>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
using namespace cv;
int minR = 0;
int minG = 0;
int minB = 0;
int maxR = 255;
int maxG = 255;
int maxB = 255;
void imageCallback(const sensor_msgs::ImageConstPtr& color_img)
{
    cv_bridge::CvImagePtr img_ptr;
    cv::Mat img_rgb;
    try
    {
        img_ptr = cv_bridge::toCvCopy(color_img,
        sensor_msgs::image_encodings::BGR8);
        img_rgb = img_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: \%s", e.what());
        return;
    }
    cvCreateTrackbar("min R:","trackbar",&minR, 255);
    cvCreateTrackbar("min G:","trackbar",&minG, 255);
    cvCreateTrackbar("min B:","trackbar",&minB, 255);
    cvCreateTrackbar("max R:","trackbar",&maxR, 255);
    cvCreateTrackbar("max G:","trackbar",&maxG, 255);
    cvCreateTrackbar("max B:","trackbar",&maxB, 255);
    //cv::Mat img_hsv;
    cv::Mat img_binary;
    CvMoments colorMoment;
    cv::Scalar min_vals(minR, minG, minB);
    cv::Scalar max_vals(maxR, maxG, maxB);
    //cv::cvtColor(img_rgb, img_hsv, CV_BGR2HSV);
    cv::inRange(img_rgb, min_vals, max_vals, img_binary);
    dilate( img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
    /*======================= TOA DO ================================*/
    colorMoment = moments(img_binary);
    double moment10 = cvGetSpatialMoment(&colorMoment, 1, 0);
    double moment01 = cvGetSpatialMoment(&colorMoment, 0, 1);
    double area = cvGetCentralMoment(&colorMoment, 0, 0);
    float posX = (moment10/area);
    float posY = (moment01/area);
    /*================= HIEN THI =================================*/
    printf("1. x-Axis  %f  y-Axis  %f  Area  %f\n", moment10, moment01, area);
    printf("2. x  %f  y %f \n\n", posX , posY);

    cv::imshow("TRACKING COLOR", img_binary);
    cv::imshow("RGB image", img_rgb);
    cv::waitKey(3);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "HSV_image");
    ros::NodeHandle nh;
    cvNamedWindow("TRACKING COLOR", 2 );
    cvNamedWindow("RGB image", 2 );
    cvNamedWindow ("trackbar", 2 );
    cvStartWindowThread();
    image_transport::ImageTransport it(nh);
    ros::Subscriber cam_img_sub =nh.subscribe("/gscam/image_raw", 1, &imageCallback);
    ros::spin();
    cvDestroyWindow("TRACKING COLOR");
    cvDestroyWindow("RGB image");
}
