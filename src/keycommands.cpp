#include <ros/ros.h>
#include <keyboard/Key.h>
#include <std_msgs/Bool.h>
// #include <controller_manager_msgs/SwitchController.h>
// #include <std_srvs/Empty.h>


uint key_pressed;

void getKeyCallback(const keyboard::Key::ConstPtr& msg)
{
  key_pressed = msg->code;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GetCommandsKeyboard");
  ros::NodeHandle node;

  ROS_INFO("[GetCommandsKeyboard] Node is ready");

  double spin_rate = 100;
  ros::param::get("~spin_rate", spin_rate);
  ROS_DEBUG( "Spin Rate %lf", spin_rate);

  key_pressed = keyboard::Key::KEY_UNKNOWN;

  ros::Subscriber sub_keyboard = node.subscribe("keyboard/keydown", 10, getKeyCallback);

  std_msgs::Bool bool_msg;
  ros::Publisher pub_reset = node.advertise<std_msgs::Bool>("/robot", 10);
  ros::Publisher pub_start_robots = node.advertise<std_msgs::Bool>("/stopandgo", 10);

  ros::Rate rate(spin_rate);

  while (node.ok())
    {
      // ROS_INFO_STREAM("DENTRO WHILE");
      ros::spinOnce();

      switch (key_pressed) 
      {

          case keyboard::Key::KEY_r: // Mosso il robot
            bool_msg.data = true;
            pub_reset.publish(bool_msg);
            ROS_INFO_STREAM("Mosso il robot");
            break;
          case keyboard::Key::KEY_g: //Start 
            bool_msg.data = true;
             pub_reset.publish(bool_msg);
             ROS_INFO_STREAM("Starting ");
            break;
          case keyboard::Key::KEY_s: //Stop Robots
            bool_msg.data = false;
            pub_start_robots.publish(bool_msg);
            ROS_INFO_STREAM("Stopping Robots");
            break;
      }

      key_pressed = keyboard::Key::KEY_UNKNOWN;
      ros::spinOnce();
      rate.sleep();
    }

  return 0;
}
