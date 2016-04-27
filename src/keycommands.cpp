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


  // std::string control_topic_right, control_topic_left;
  // node.param<std::string>("/right_arm/controller", control_topic_right, "PotentialFieldControlKinematicReverse");
  // node.param<std::string>("/left_arm/controller", control_topic_left, "PotentialFieldControlKinematicReverse");
  // ros::ServiceClient client;
  // std_srvs::Empty empty_msgs;

  // double right_arm_max_vel_percentage, right_arm_vel_limit_robot;
  // double left_arm_max_vel_percentage, left_arm_vel_limit_robot;


  // bool right_arm_enabled, left_arm_enabled;


  ros::Rate rate(spin_rate);

  while (node.ok())
    {

      // node.param<bool>("/right_arm/enabled", right_arm_enabled, "false");
      // node.param<bool>("/left_arm/enabled", left_arm_enabled, "false");

      ros::spinOnce();

      switch (key_pressed) {

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
        // case keyboard::Key::KEY_KP_PLUS:
        //   node.param<double>("/right_arm/" + control_topic_right +"/max_vel_percentage", right_arm_max_vel_percentage, 0.5);
        //   node.param<double>("/right_arm/" + control_topic_right +"/vel_limit_robot", right_arm_vel_limit_robot , 0.5);
        //   node.param<double>("/left_arm/" + control_topic_left +"/max_vel_percentage", left_arm_max_vel_percentage, 0.5);
        //   node.param<double>("/left_arm/" + control_topic_left +"/vel_limit_robot", left_arm_vel_limit_robot , 0.5);

        //   node.setParam("/right_arm/" + control_topic_right +"/max_vel_percentage", right_arm_max_vel_percentage + 0.0*.05);
        //   node.setParam("/right_arm/" + control_topic_right +"/vel_limit_robot", right_arm_vel_limit_robot + 0.1);
        //   node.setParam("/left_arm/" + control_topic_left +"/max_vel_percentage", left_arm_max_vel_percentage + 0.0*.05);
        //   node.setParam("/left_arm/" + control_topic_left +"/vel_limit_robot", left_arm_vel_limit_robot + 0.1);
        //   client = node.serviceClient<std_srvs::Empty>("/right_arm/" + control_topic_right + "/load_velocity");
        //   if (right_arm_enabled)
        //     {
        //       client.call(empty_msgs);
        //       ROS_INFO_STREAM("Increasing velocity on right_arm/" << control_topic_right.c_str());
        //     }
        //   client = node.serviceClient<std_srvs::Empty>("/left_arm/" + control_topic_right + "/load_velocity");
        //   if (left_arm_enabled)
        //     {
        //       client.call(empty_msgs);
        //       ROS_INFO_STREAM("Increasing velocity on left_arm/" << control_topic_left.c_str());
        //     }

        //   break;
        // case keyboard::Key::KEY_KP_MINUS:
        //   node.param<double>("/right_arm/" + control_topic_right +"/max_vel_percentage", right_arm_max_vel_percentage, 0.5);
        //   node.param<double>("/right_arm/" + control_topic_right +"/vel_limit_robot", right_arm_vel_limit_robot , 0.5);
        //   node.param<double>("/left_arm/" + control_topic_left +"/max_vel_percentage", left_arm_max_vel_percentage, 0.5);
        //   node.param<double>("/left_arm/" + control_topic_left +"/vel_limit_robot", left_arm_vel_limit_robot , 0.5);

        //   node.setParam("/right_arm/" + control_topic_right +"/max_vel_percentage", right_arm_max_vel_percentage - 0.0*.05);
        //   node.setParam("/right_arm/" + control_topic_right +"/vel_limit_robot", right_arm_vel_limit_robot - 0.1);
        //   node.setParam("/left_arm/" + control_topic_left +"/max_vel_percentage", left_arm_max_vel_percentage - 0.0*.05);
        //   node.setParam("/left_arm/" + control_topic_left +"/vel_limit_robot", left_arm_vel_limit_robot - 0.1);
        //   client = node.serviceClient<std_srvs::Empty>("/right_arm/" + control_topic_right + "/load_velocity");
        //   if (right_arm_enabled)
        //     {
        //       client.call(empty_msgs);
        //       ROS_INFO_STREAM("Decreasing velocity on right_arm/" << control_topic_right.c_str());
        //     }
        //   client = node.serviceClient<std_srvs::Empty>("/left_arm/" + control_topic_right + "/load_velocity");
        //   if (left_arm_enabled)
        //     {
        //       client.call(empty_msgs);
        //       ROS_INFO_STREAM("Decreasing velocity on left_arm/" << control_topic_left.c_str());
        //     }

        //   break;
        // case keyboard::Key::KEY_i: //Stop Robots
        //   ROS_INFO_STREAM("Initializing Controllers");
        //   controller_manager_msgs::SwitchController switch_srv;
        //   switch_srv.request.start_controllers.push_back(control_topic_right);
        //   switch_srv.request.stop_controllers.push_back("joint_trajectory_controller");
        //   switch_srv.request.strictness = 2;

        //   client = node.serviceClient<controller_manager_msgs::SwitchController>("/right_arm/controller_manager/switch_controller");
        //   if (right_arm_enabled)
        //     {
        //       if (!client.call(switch_srv))
        //         {
        //           ROS_ERROR_STREAM("Not possible to switch right_arm control to: " << control_topic_right.c_str());
        //         }
        //       else
        //         {
        //           if (switch_srv.response.ok == true)
        //             {
        //               ROS_INFO_STREAM("right_arm control switched to: " << control_topic_right.c_str());
        //             }
        //           else
        //             {
        //               ROS_ERROR_STREAM("Not possible to switch right_arm control to: " << control_topic_right.c_str());
        //             }
        //         }
        //     }
        //   switch_srv.request.start_controllers.push_back(control_topic_left);
        //   client = node.serviceClient<controller_manager_msgs::SwitchController>("/left_arm/controller_manager/switch_controller");
        //   if (left_arm_enabled)
        //     {
        //       if (!client.call(switch_srv))
        //         {
        //           ROS_ERROR_STREAM("Not possible to switch right_arm control to: " << control_topic_right.c_str());
        //         }
        //       else
        //         {
        //           if (switch_srv.response.ok == true)
        //             {
        //               ROS_INFO_STREAM("left_arm control switched to: " << control_topic_right.c_str());
        //             }
        //           else
        //             {
        //               ROS_ERROR_STREAM("Not possible to switch left_arm control to: " << control_topic_right.c_str());
        //             }
        //         }
        //     }
        //   break;


        }
      key_pressed = keyboard::Key::KEY_UNKNOWN;
      ros::spinOnce();
      rate.sleep();
    }

  return 0;
}
