#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <string>
#include "ros/console.h"
#include <std_msgs/Int32.h>
/**
* @简短的Line Detect类包含用于图像处理和方向发布的所有功能
*/
class LineDetect {
public:
    cv::Mat img;  /// 以OpenCV矩阵格式输入图像
    cv::Mat img_filt;  /// OpenCV矩阵格式的过滤图像
    int dir;  /// 要发布的指导消息
/**
* @ brief回调用于从Turtlebot订阅图像主题并将其转换为opencv图像格式
* @ param msg是ROS的图像消息
* @不返回
*/
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
/**
*@brief Function that applies Gaussian filter in the input image
*@param input is the image from the turtlebot in opencv matrix format
*@return Mat of Gaussian filtered image in opencv matrix format
* @ brief函数在输入图像中应用高斯滤波器
* @ param输入是来自turtlebot的图像，格式为opencv
* @ opencv矩阵格式的高斯滤波图像的返回垫
*/
    cv::Mat Gauss(cv::Mat input);
/**
*@brief Function to perform line detection using color thresholding,image masking and centroid detection to publish direction
*@param input is the Filtered input image in opencv matrix format
*@return int direction which returns the direction the turtlebot should head in
* @ brief功能使用颜色阈值，图像蒙版和质心检测执行发布线检测以发布方向
* @ param input是opencv矩阵格式的过滤后的输入图像
* @ return int direction返回乌龟应该进入的方向
*/
    int colorthresh(cv::Mat input);

private:
    cv::Scalar LowerYellow;
    cv::Scalar UpperYellow;
    cv::Mat img_hsv;
    cv::Mat img_mask;
};


void LineDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr->image;
        //cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
// Convert input image to HSV
//cv::Mat image = cv_ptr->image;
//cv::Mat hsvImage;
int LineDetect::colorthresh(cv::Mat img){
    cv::cvtColor(LineDetect::img, LineDetect::img_hsv, cv::COLOR_BGR2HSV);

    // Threshold the HSV image, keep only the yellow pixels
    LineDetect::LowerYellow = {20, 100, 100};
    LineDetect::UpperYellow = {30, 255, 255};


    //cv::Mat mask;
    cv::inRange(LineDetect::img_hsv, LineDetect::LowerYellow, LineDetect::UpperYellow, LineDetect::img_mask);

    int width = LineDetect::img_mask.cols;
    int height = LineDetect::img_mask.rows;

    int search_top = 3 * height / 4;
    int search_bottom = search_top + 10;

    // Zero out pixels outside the desired region
    for (int y = 0; y < height - 2; y++) // For some reason, cannot modify the last two rows
    {
        if (y < search_top || y > search_bottom) {
            for (int x = 0; x < width; x++) {
                LineDetect::img_mask.at<cv::Vec3b>(y, x)[0] = 0;
                LineDetect::img_mask.at<cv::Vec3b>(y, x)[1] = 0;
                LineDetect::img_mask.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }
    auto count = 0;
    // Use the OpenCV moments() function to calculate the centroid of the blob of the binary image
    cv::Moments M = cv::moments(LineDetect::img_mask);
    /*std::vector<std::vector<cv::Point> > v;
    cv::findContours(LineDetect::img_mask, v, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    cv::drawContours(LineDetect::img,v,-1,(0,255,0),5);
    if (v.size() != 0) {
        auto area = 0;
        auto idx = 0;

        while (count < v.size()) {
            if (area < v[count].size()) {
                idx = count;
                area = v[count].size();
            }
            count++;
        }
    }*/
        //if (count == 0) {
         //   LineDetect::dir = 2000;
           // ROS_INFO("%d",LineDetect::dir);
        //} else {
            if (M.m00 > 0) {
                int cx = int(M.m10 / M.m00);
                int cy = int(M.m01 / M.m00);
                cv::circle(LineDetect::img, cv::Point(cx, cy), 20, CV_RGB(255, 0, 0), -1);
                cv::imshow("test",LineDetect::img);
                cv::waitKey(10);
                // Move the robot in proportion to the error signal
                LineDetect::dir = cx;
                //ROS_INFO("%d",LineDetect::dir);
            }
        //}
        return LineDetect::dir;
    }




/*cv::Mat LineDetect::Gauss(cv::Mat input) {
    cv::Mat output;
// Applying Gaussian Filter应用高斯滤波器
    cv::GaussianBlur(input, output, cv::Size(3, 3), 0.1, 0.1);
    return output;
}

int LineDetect::colorthresh(cv::Mat input) {
    // Initializaing variables初始化变量
    cv::Size s = input.size();
    std::vector<std::vector<cv::Point> > v;
    auto w = s.width;
    auto h = s.height;
    auto c_x = 0.0;
    // Detect all objects within the HSV range RGBToHSV 检测HSV范围RGBToHSV内的所有对象
    cv::cvtColor(input, LineDetect::img_hsv, cv::COLOR_BGR2HSV);
    LineDetect::LowerYellow = {0, 252, 255};//Red
    LineDetect::UpperYellow = {0, 255, 255};
    //LineDetect::LowerYellow = {100, 255, 100};//Blue
    //LineDetect::UpperYellow = {100, 255, 255};
    //LineDetect::LowerYellow = {20, 100, 100};//Yellow
    //LineDetect::UpperYellow = {30, 255, 255};
    cv::inRange(LineDetect::img_hsv, LowerYellow,
                UpperYellow, LineDetect::img_mask);
    img_mask(cv::Rect(0, 0, w, 0.8*h)) = 0;
    // Find contours for better visualization 查找轮廓以获得更好的可视化
    cv::findContours(LineDetect::img_mask, v, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    // If contours exist add a bounding  如果存在轮廓，则添加边界
    // Choosing contours with maximum area  选择最大面积的轮廓
    if (v.size() != 0) {
        auto area = 0;
        auto idx = 0;
        auto count = 0;
        while (count < v.size()) {
            if (area < v[count].size()) {
                idx = count;
                area = v[count].size();
            }
            count++;
        }
        cv::Rect rect = boundingRect(v[idx]);
        cv::Point pt1, pt2, pt3;
        pt1.x = rect.x;
        pt1.y = rect.y;
        pt2.x = rect.x + rect.width;
        pt2.y = rect.y + rect.height;
        pt3.x = pt1.x+5;
        pt3.y = pt1.y-5;
        // Drawing the rectangle using points obtained  使用获得的点绘制矩形
        rectangle(input, pt1, pt2, CV_RGB(255, 0, 0), 2);
        // Inserting text box 插入文字框
        cv::putText(input, "Line Detected", pt3,
                    cv::FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
    }
    // Mask image to limit the future turns affecting the output 遮罩图像以限制将来的转向影响输出
    img_mask(cv::Rect(0.7*w, 0, 0.3*w, h)) = 0;
    img_mask(cv::Rect(0, 0, 0.3*w, h)) = 0;
    // Perform centroid detection of line  执行线的质心检测
    cv::Moments M = cv::moments(LineDetect::img_mask);
    if (M.m00 > 0) {
        cv::Point p1(M.m10/M.m00, M.m01/M.m00);
        cv::circle(LineDetect::img_mask, p1, 5, cv::Scalar(155, 200, 0), -1);
    }
    c_x = M.m10/M.m00;
    // Tolerance to chooise directions  容忍选择方向
    auto tol = 15;
    auto count = cv::countNonZero(img_mask);
    // Turn left if centroid is to the left of the image center minus tolerance
    // Turn right if centroid is to the right of the image center plus tolerance
    // Go straight if centroid is near image center
    //如果质心在图像中心的左侧减去公差，则向左转
    //如果质心在图像中心的右侧加上公差，则向右转
    //如果质心在图像中心附近，请直走
    if (c_x < w/2-tol) {
        LineDetect::dir = 0;
    } else if (c_x > w/2+tol) {
        LineDetect::dir = 2;
    } else {
        LineDetect::dir = 1;
    }
    // Search if no line detected 搜索是否未检测到行
    if (count == 0) {
        LineDetect::dir = 3;
    }
    // Output images viewed by the turtlebot 输出乌龟机器人查看的图像
    //cv::namedWindow("Turtlebot View");
    //imshow("Turtlebot View", input);
    return LineDetect::dir;
}
*/


int main(int argc, char **argv) {
    // Initializing node and object初始化节点和对象
    ros::init(argc, argv, "detection");
    ros::NodeHandle n;
    LineDetect det;
    det.dir=2000;

    // Creating Publisher and subscriber创建发布者和订阅者
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw",1, &LineDetect::imageCallback, &det);
    //ros::Subscriber sub = n.subscribe("/usb_cam/image_raw",
    //    1, &LineDetect::imageCallback, &det);

    ros::Publisher dirPub = n.advertise<std_msgs::Int32>("/line_side", 10);
    std_msgs::Int32 msg;

    while (ros::ok()) {
        if (!det.img.empty()) {
            //cv::imshow("test",det.img);
            //cv::waitKey(10);
            // Perform image processing执行图像处理
            //det.img_filt = det.Gauss(det.img);
            msg.data = det.colorthresh(det.img);
            // Publish direction message发布指导消息
            dirPub.publish(msg);
        }

        ros::spinOnce();
    }
    // Closing image viewer关闭图像查看器
    //cv::destroyWindow("Turtlebot View");
    return 0;
}