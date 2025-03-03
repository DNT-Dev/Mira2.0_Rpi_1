#include <custom_msgs/commands.h>
#include <custom_msgs/telemetry.h>
#include <mira2_pid_control/control_utils.hpp>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32MultiArray.h>
/*
    Bot Orientation
    +ve   x  -ve (CCW)
    (CW)  |
    y -- bot
    +ve
*/

// Predefined Variables
#define threshold 8 // degrees

// Control parameters and PWM Commands
bool                  software_arm_flag = false;
custom_msgs::commands cmd_pwm;
PID_Controller        depth, yaw;
double                depth_error, depth_setpoint;
int                   yaw_error, yaw_setpoint;
ros::Time             start_routine;
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
    } else if (key == 'w') {
        depth.kp = depth.kp + 0.2;
        std::cout << "current depth kp value: " + std::to_string(depth.kp)
                  << std::endl;
    } else if (key == 's') {
        depth.kp = depth.kp - 0.05;
        std::cout << "current depth kp value: " + std::to_string(depth.kp)
                  << std::endl;
    } else if (key == 'e') {
        depth.ki = depth.ki + 0.005;
        std::cout << "current depth ki value: " + std::to_string(depth.ki)
                  << std::endl;
    } else if (key == 'd') {
        depth.ki = depth.ki - 0.001;
        std::cout << "current depth ki value: " + std::to_string(depth.ki)
                  << std::endl;
    } else if (key == 'r') {
        depth.kd = depth.kd + 0.1;
        std::cout << "current depth kd value: " + std::to_string(depth.kd)
                  << std::endl;
    } else if (key == 'f') {
        depth.kd = depth.kd - 0.1;
        std::cout << "current depth kd value: " + std::to_string(depth.kd)
                  << std::endl;
    }
}

void telemetryCallback(const custom_msgs::telemetry::ConstPtr &msg) {
    bool   armed          = msg->arm; // useless
    double depth_external = msg->external_pressure;
    int    yaw_heading    = msg->heading;
    // if (depth_external < RESETDEPTH)
    // {
    //     reseting = false;
    // }
    depth_error = depth_setpoint - depth_external;
    yaw_error = yaw_setpoint - yaw_heading;
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

    // Control Parameters Definition

    // Depth
    depth.kp = 0;     
    depth.ki = 0;   
    depth.kd = 0; 
    depth.base_offset = 1580;
    depth_setpoint = 1069;

    yaw.kp = 0;     
    yaw.ki = 0;   
    yaw.kd = 0; 
    yaw.base_offset = 1500; //this is 1500 only
    yaw_setpoint = 90;   //see headings dumdums


    // Arm Disarm Parameter
    bool      arm       = false;
    ros::Time init_time = ros::Time::now();
    cmd_pwm.arm         = false;

    while (ros::ok()) {
        if (software_arm_flag == true) {
            cmd_pwm.mode       = "STABILIZE";
            ros::Time time_now = ros::Time::now();
            cmd_pwm.arm        = true;
            if (software_arm_flag == true) {
                float pid_depth = depth.pid_control(
                    depth_error, (time_now - init_time).toSec(), false);
                
                float pid_yaw = yaw.pid_control(
                    yaw_error, (time_now - init_time).toSec(), false);
                std::cout <<" "<<pid_depth<<" "<<pid_yaw<<"\n";  // ((time_now - start_routine).toSec()) << "\n";
                cmd_pwm.forward = 1500;
                cmd_pwm.lateral = 1500;
                cmd_pwm.thrust  = pid_depth;
                cmd_pwm.yaw     = pid_yaw;
                std::cout << "sinking";
            }
        } else {
            depth.emptyError();
            cmd_pwm.arm     = false;
            cmd_pwm.mode    = "STABILIZE";
            cmd_pwm.forward = 1500;
            cmd_pwm.lateral = 1500;
            cmd_pwm.thrust  = 1500;
            cmd_pwm.yaw     = 1500;
        }
        pwm_publisher.publish(cmd_pwm);

        ros::spinOnce();
    }
}