#include<ros/ros.h>
#include<signal.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Int32.h>
#include<time.h>
#include<stdio.h>
ros::Publisher g_cmdVelPub;
//全局变量加个g_
int g_ball_x = 2000;
//小球在图像中的横坐标
int g_flag_yaw = 1;
//先让小车装转向指定方位
double p_error = 0;
double i_error = 0;
double d_error = 0;
double Kp = 0.0006;
double Ki = 0;
double Kd = 0.0001;
double pre_diff = 0;

void Shutdown(int sig) {
  g_cmdVelPub.publish(geometry_msgs::Twist());
  ROS_INFO("goforward cpp ended!");
  ros::shutdown();
}
//使机器人停止运动
void DoMsg(const std_msgs::Int32::ConstPtr &msg) {
  ROS_INFO("visual=%d", msg->data);
  g_ball_x = msg->data;
}
//更新小球在图像中的横坐标
double PidControl(double diff) {
  p_error = diff;
  i_error = i_error + diff;
  d_error = diff - pre_diff;
  pre_diff = diff;
  ROS_INFO("PidResult=%.6f", Kp * p_error + Ki * i_error + Kd * d_error);
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "GoForward");
  //初始化ROS,它允许ROS通过命令行进行名称重映射
  std::string topic = "/cmd_vel";
  ros::NodeHandle n;
  //为这个进程的节点创建一个句柄
  ros::Subscriber sub = n.subscribe<std_msgs::Int32>("/bool_ball", 1, DoMsg);
  g_cmdVelPub = n.advertise<geometry_msgs::Twist>(topic, 1);
  //告诉master将要在/cmd_vel topic上发布一个geometry_msgs/Twist的消息
  ros::Rate r(10);
  //ros::Rate对象可以允许你指定自循环的频率
  signal(SIGINT, Shutdown);
  //在程序终止（ctrl+c）后使得小车停下
  ROS_INFO("goforward cpp start...");
  geometry_msgs::Twist speed;
  // 控制信号载体Twist messag
  while (ros::ok()) {
    if (g_flag_yaw == 0) {
      g_flag_yaw++;
      speed.angular.z = -0.6;
      speed.linear.x = 0;
      ROS_INFO("in");
      ros::Duration(0.1).sleep();
      ROS_INFO("in");
      g_cmdVelPub.publish(speed);
      ros::Duration(2.6).sleep();
      //奇怪的问题？？？？
      //要先向话题发布一次命令后下一次才会生效
      ROS_INFO("out");
      //先调整一下车的朝向
    }
    ros::Time begin = ros::Time::now();
    time_t first;
    first = time(NULL);
    while (difftime(time(NULL), first) <= 2 * 3.5 / 0.6) {
      //判定还没转到一圈
      if (g_ball_x == 2000) { //视野内无小球
        speed.angular.z = -0.6;  // 设置角速度为0rad/s，正为左转，负为右转
        speed.linear.x = 0;       // 设置线速度为0.1m/s，正为前进，负为后退
        g_cmdVelPub.publish(speed);
      } else {         //视野内有小球
        speed.linear.x = 0.6;
        speed.angular.z = PidControl(960.0 - g_ball_x);
        g_cmdVelPub.publish(speed);
        break;
      }
      ros::spinOnce();
    }
    if (difftime(time(NULL), first) > 2 * 3.5 / 0.6) {
      //转圈超过一圈 向前走一点距离
      speed.linear.x = 0.3;
      speed.angular.z = 0;
      double time_one_circle = 2 / speed.linear.x;
      g_cmdVelPub.publish(speed);
      ros::Duration(time_one_circle).sleep();
    }
    g_cmdVelPub.publish(speed);         //  将刚才设置的指令发送给机器人
    ros::spinOnce();                  //  调用回调函数处理订阅到的消息
    r.sleep();                        //  休眠直到一个频率周期的时间
  }
  //判断是否撞到小球？
  return 0;
}

