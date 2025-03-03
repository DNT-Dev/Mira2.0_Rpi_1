#include <custom_msgs/commands.h>
#include <custom_msgs/telemetry.h>
#include <mira2_pid_control/control_utils.hpp>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32MultiArray.h>

// Control parameters and PWM Commands
bool software_arm_flag = false;
custom_msgs::commands cmd_pwm;
PID_Controller depth;
double depth_error;
ros::Time start_routine;
/* Keys callback
    Function for tuning the PID parameters
*/
void keys_callback(const std_msgs::Char::ConstPtr &msg) {
  char key = msg->data;
  if (key == 'q') {
    software_arm_flag = false;
    // cmd_pwm.arm = false;
    std::cout << "unarmed\n";
    start_routine = ros::Time::now();
    depth.emptyError();
  } else if (key == 'p') {
    software_arm_flag = true;
    // cmd_pwm.arm = true;
    std::cout << "armed\n";
    start_routine = ros::Time::now();
  }
}

void telemetryCallback(const custom_msgs::telemetry::ConstPtr &msg) {
  double depth_external = msg->external_pressure;
  depth_error = 1075 - depth_external;
}

int main(int argc, char **argv) {
  // ROS Node Declaration
  ros::init(argc, argv, "depth_tuner_controller");
  ros::NodeHandle nh;

  // ROS Publisher
  ros::Publisher pwm_publisher =
      nh.advertise<custom_msgs::commands>("/master/commands", 1);

  // ROS Subscriber
  ros::Subscriber keys_subscriber = nh.subscribe("keys", 1, keys_callback);

  ros::Subscriber telemetry_sub =
      nh.subscribe("/master/telemetry", 1, telemetryCallback);

  // Depth
  depth.kp = -2;     //-2;
  depth.ki = -0.2;   //-0.2;
  depth.kd = -10.69; //-15.69;

  printf("lilbitchlaky\n");

  // Arm Disarm Parameter
  bool arm = false;
  ros::Time init_time = ros::Time::now();
  cmd_pwm.arm = false;

  while (ros::ok()) {
    if (software_arm_flag == true) {
      cmd_pwm.mode = "STABILIZE";
      ros::Time time_now = ros::Time::now();
      cmd_pwm.arm = false;
      if (software_arm_flag == true) {
        float pid_depth = depth.pid_control(
            depth_error, (time_now - init_time).toSec(), false);
        std::cout << ((time_now - start_routine).toSec()) << "\n";
        float delay = 69.0;
        if ((time_now - start_routine).toSec() < delay) {
          cmd_pwm.arm = false;
	  cmd_pwm.forward = 1500;
	  cmd_pwm.mode = "STABILIZE";
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          depth.emptyError();
        } else if ((time_now - start_routine).toSec() < (delay + 5)) {
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          std::cout << "sinking ";
        } else if ((time_now - start_routine).toSec() < (delay + 10.50)) {
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1800;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          std::cout << "stabilize surge ";
        } else if ((time_now - start_routine).toSec() < (delay + 13.10)) {
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1800;
          std::cout << "stabilize yaw ";
        } else if ((time_now - start_routine).toSec() < (delay + 16.30)) {
          cmd_pwm.arm = true;
          cmd_pwm.forward = 1800;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          std::cout << "stabilize surge ";
        } else if ((time_now - start_routine).toSec() < (delay + 16.35)) {
          cmd_pwm.arm = false;
          cmd_pwm.mode = "STABILIZE";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "disarm";
        } else if ((time_now - start_routine).toSec() < (delay + 16.40)) {
          cmd_pwm.arm = false;
          cmd_pwm.mode = "MANUAL";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "change to manual";
        } else if ((time_now - start_routine).toSec() < (delay + 19.50)) {
          cmd_pwm.arm = true;
          cmd_pwm.mode = "MANUAL";
          cmd_pwm.forward = 1800;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          cmd_pwm.roll = 1850;
          std::cout << " surge with roll  and manual";
        } else if ((time_now - start_routine).toSec() < (delay + 19.55)) {
          cmd_pwm.arm = false;
          cmd_pwm.mode = "MANUAL";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "disarm";
        } else if ((time_now - start_routine).toSec() < (delay + 19.60)) {
          cmd_pwm.arm = false;
          cmd_pwm.mode = "STABILIZE";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = 1500;
          cmd_pwm.yaw = 1500;
          std::cout << "change to stqbilize";
        } else if ((time_now - start_routine).toSec() < (delay + 21.60)) {
          cmd_pwm.arm = true;
          cmd_pwm.mode = "STABILIZE";
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          cmd_pwm.roll = 1500;
          std::cout << "forward";
        } else if((time_now -start_routine).toSec()<(delay + 25)){
          cmd_pwm.arm= false;
          cmd_pwm.forward= 1500;
          cmd_pwm.lateral=1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw=1500;
          std::cout<<"wait";
        } else {
          cmd_pwm.arm = false;
          cmd_pwm.forward = 1500;
          cmd_pwm.lateral = 1500;
          cmd_pwm.thrust = pid_depth;
          cmd_pwm.yaw = 1500;
          std::cout << "pusi";
        }
      }
    } else {
      depth.emptyError();
      cmd_pwm.arm = false;
      cmd_pwm.mode = "STABILIZE";
      cmd_pwm.forward = 1500;
      cmd_pwm.lateral = 1500;
      cmd_pwm.thrust = 1500;
      cmd_pwm.yaw = 1500;
      
    }
    pwm_publisher.publish(cmd_pwm);
    ros::spinOnce();
  }
}

// if (software_arm_flag == true) {
//             cmd_pwm.mode       = "STABILIZE";
//             ros::Time time_now = ros::Time::now();
//             cmd_pwm.arm        = true;
//             if (software_arm_flag == true) {
//                 float pid_depth = depth.pid_control(
//                     depth_error, (time_now - init_time).toSec(), false);
//                 std::cout << ((time_now - start_routine).toSec()) << "\n";
//                 if ((time_now - start_routine).toSec() < 3.0) {
//                     cmd_pwm.forward = 1500;
//                     cmd_pwm.lateral = 1500;
//                     cmd_pwm.thrust  = pid_depth;
//                     cmd_pwm.yaw     = 1500;
//                     std::cout << "sinking";
//                 } else if ((time_now - start_routine).toSec() < 8.0) {
//                     cmd_pwm.forward = 1800;
//                     cmd_pwm.lateral = 1500;
//                     cmd_pwm.thrust  = pid_depth;
//                     cmd_pwm.yaw     = 1500;
//                     std::cout << "forward";
//                 } else if ((time_now - start_routine).toSec() < 10.25) {
//                     cmd_pwm.forward = 1500;
//                     cmd_pwm.lateral = 1500;
//                     cmd_pwm.thrust  = pid_depth;
//                     cmd_pwm.yaw     = 1800;
//                     std::cout << "yaw";
//                 } else if ((time_now - start_routine).toSec() < 15.25) {
//                     cmd_pwm.forward = 1800;
//                     cmd_pwm.lateral = 1500;
//                     cmd_pwm.thrust  = pid_depth;
//                     cmd_pwm.yaw     = 1500;
//                     std::cout << "forward";
//                 } else {
//                     cmd_pwm.forward = 1500;
//                     cmd_pwm.lateral = 1500;
//                     cmd_pwm.thrust  = pid_depth;
//                     cmd_pwm.yaw     = 1500;
//                     std::cout << "pusi";
//                 }
//             }
//         } else {
//             depth.emptyError();
//             cmd_pwm.arm     = false;
//             cmd_pwm.mode    = "STABILIZE";
//             cmd_pwm.forward = 1500;
//             cmd_pwm.lateral = 1500;
//             cmd_pwm.thrust  = 1500;
//             cmd_pwm.yaw     = 1500;
//         }
