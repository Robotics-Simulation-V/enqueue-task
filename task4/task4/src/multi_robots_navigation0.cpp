#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//move_base_msgs::MoveBaseAction是move_base在world中的目标
//rostopic info /move_base/goal 的数据类型是move_base_msgs/MoveBaseActionGoal
//所以不能使用之前那种 geometry_msgs::Twist

int main(int argc, char **argv) {
  ros::init(argc, argv, "send_goals_node");
  MoveBaseClient ac("/robot1/move_base", true);
  //创建action客户端，参数1：action名，参数2：true，不需要手动调用ros::spin()，会在它的线程中自动调用。
  ROS_INFO("Waiting for the move_base action server");
  ac.waitForServer(ros::Duration(60));
  // 等待60s
  ROS_INFO("Connected to move base server");
  //目标的属性设置

  while (ros::ok()) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "robot2_tf/base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = -0.3;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.orientation.w = 1;
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    // 向move_base发送目的地属性
    ac.waitForResult(ros::Duration(0.1);
    // 等待响应
  }
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("get it!");
  else
    ROS_INFO("failed to get it");
  return 0;
}