#include<opencv4/opencv2/opencv.hpp>
#include<opencv2/highgui/highgui_c.h>
#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<geometry_msgs/Twist.h>
#include<cv_bridge/cv_bridge.h>
#include<std_msgs/Int32.h>
#include<time.h>
#include<chrono>

using namespace cv;
using namespace std;
int value = 2000;

int Hmin = 45, Smin = 43, Vmin = 46;
int Hmax = 90, Smax = 255, Vmax = 255;

void image_Callback(const sensor_msgs::Image &msg);

void printer_value(int, void *);

int main(int argc, char **argv) {
  namedWindow("RGB调节", (640, 200));
  createTrackbar("Hlow", "RGB调节", nullptr, 180, printer_value);
  setTrackbarPos("Hlow", "RGB调节", Hmin);

  createTrackbar("Hmax", "RGB调节", nullptr, 180, printer_value);
  setTrackbarPos("Hmax", "RGB调节", Hmax);

  createTrackbar("Slow", "RGB调节", nullptr, 255, printer_value);
  setTrackbarPos("Slow", "RGB调节", Smin);

  createTrackbar("Smax", "RGB调节", nullptr, 255, printer_value);
  setTrackbarPos("Smax", "RGB调节", Smax);

  createTrackbar("Vlow", "RGB调节", nullptr, 255, printer_value);
  setTrackbarPos("Vlow", "RGB调节", Vmin);

  createTrackbar("Vmax", "RGB调节", nullptr, 255, printer_value);
  setTrackbarPos("Vmax", "RGB调节", Vmax);

  ros::init(argc, argv, "identify_balls");
  ros::NodeHandle nh;
  ros::Subscriber img_sub = nh.subscribe("/camera/rgb/image_raw", 1, image_Callback);
  ros::Publisher int_pub = nh.advertise<std_msgs::Int32>("/bool_ball", 1);
  while (ros::ok()) {
    waitKey(1);
    std_msgs::Int32 msg;
    msg.data = value;
    int_pub.publish(msg);
    ros::spinOnce();
  }
}

void image_Callback(const sensor_msgs::Image &msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat frame = cv_ptr->image;
  //auto begin = std::chrono::system_clock::now();
  cv::Mat hsv = frame.clone();
  cv::Mat midImage = frame.clone();
  //auto begin = std::chrono::system_clock::now();
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  Hmin = getTrackbarPos("Hlow", "RGB调节");
  Smin = getTrackbarPos("Slow", "RGB调节");
  Vmin = getTrackbarPos("Vlow", "RGB调节");
  Hmax = getTrackbarPos("Hmax", "RGB调节");
  Smax = getTrackbarPos("Smax", "RGB调节");
  Vmax = getTrackbarPos("Vmax", "RGB调节");

  inRange(hsv, Scalar(Hmin, Smin, Vmin), Scalar(Hmax, Smax, Vmax), midImage);
  GaussianBlur(midImage, midImage, Size(9, 9), 2, 2);//进行霍夫圆变换
  vector<Vec3f> circles;
  HoughCircles(midImage, circles, CV_HOUGH_GRADIENT, 2.5, 10, 200, 100, 0, 0);//霍夫圆变换
  //cv::imshow("test",frame);
  //waitKey(10);
  for (size_t i = 0; i < circles.size(); i++) {
    Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    circle(frame, center, 3, Scalar(0, 255, 0), -1, 8, 0);//打印圆心坐标

    //if(cvRound(circles[i][0])<=870&&cvRound(circles[i][0])>=850){
    //if(cvRound(circles[i][0])<=930&&cvRound(circles[i][0])>=920){
    //960为中值
    value = circles[i][0];
  }
  cv::imshow("test", frame);
  if (circles.size() == 0) {
    value = 2000;
  }

  //auto end = std::chrono::system_clock::now();
  //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count();
  //ROS_INFO("%d FPS",1000/duration);
}

void printer_value(int, void *) {

  printf("hmin: %d \n hmax: %d \n smin: %d \n smax: %d \n vmin: %d \n vmax: %d \n", Hmin, Hmax, Smin, Smax, Vmin,
         Vmax);
}