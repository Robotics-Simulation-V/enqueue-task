#include <iostream>
#include <string>
#include <sstream>
#include <signal.h>
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
//向小车发送运动速度
ros::Publisher g_drive_pub;

//小球在图像中的横坐标
double g_ball_x=2000;

//pid的相关的值
double p_error=0;
double i_error=0;
double d_error=0;
double Kp=0.0015;
double Ki=0.0000;
double Kd=0.001;
double pre_diff=0;

//小球在图像中的横坐标
void DoMsg(const std_msgs::Int32::ConstPtr& msg){
  g_ball_x = (double)(msg->data);   //传过来的数据一定要初始化
  ROS_INFO("difference=%.6f",g_ball_x);
}

//使机器人停止运动
void  Shutdown(int sig){
  g_drive_pub.publish(geometry_msgs::Twist ());
  ROS_INFO( "goforward cpp ended!" );
  ros::shutdown ();
}

//不断计算角速度
double PidControl(double diff)
{
  const int threshold = 100;
  if (i_error > threshold)          //误差累计过大，就消去
  {
    i_error = 0;
  }
  p_error = diff;
  i_error = i_error + diff;
  d_error = diff - pre_diff;
  pre_diff = diff;
  ROS_INFO("PIDresult=%.6f",Kp * p_error + Ki * i_error + Kd * d_error);
  return Kp * p_error + Ki * i_error + Kd * d_error;
}


int main(int argc, char** argv)
{
  //初始化ROS
  ros::init(argc, argv, "command_vel");
  //为这个进程的节点创建一个句柄
  ros::NodeHandle nh;
  //发布小车运动速度
  g_drive_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  //订阅小球在图像中的横坐标
  ros::Subscriber vision_sub=nh.subscribe<std_msgs::Int32>("/line_side",1000,DoMsg);
  //指定自循环的频率
  ros::Rate  r(10);
  //在程序终止（ctrl+c）后使得小车停下
  signal(SIGINT, Shutdown);
  //申明小车速度
  geometry_msgs::Twist drive_result;
  while (ros::ok()) {
    //视野内无小球
    if (g_ball_x==2000){
      drive_result.angular.z = 0.2 ;
      drive_result.linear.x = 0 ;
    }
    //视野内有小球
    else{
      drive_result.linear.x = 0.5 ;
      drive_result.angular.z = PidControl(960-g_ball_x);
    }
    //把速度发送给机器人
    g_drive_pub.publish(drive_result);
    //调用回调函数
    ros::spinOnce();
    //休眠一个周期的时间
    r.sleep();
  }
  return 0;
}
