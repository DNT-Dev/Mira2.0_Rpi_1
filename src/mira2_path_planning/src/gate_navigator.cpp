#include <ros/ros.h>
//#include <mira2_pid_control/control_utils.hpp>
#include "../../mira2_pid_control/include/mira2_pid_control/control_utils.hpp"
#include <custom_msgs/commands.h>
#include <custom_msgs/telemetry.h>
#include <vision_msgs/Detection2DArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

class GateNavigator {
public: 
    ros::NodeHandle         nh_;
    

    // PID Controllers
    PID_Controller depth_pid_obj;
    PID_Controller yaw_pid_obj;
    PID_Controller lateral_pid_obj;
    PID_Controller forward_pid_obj;
    

    // Current state
    int image_center_x;
    int current_yaw;

    
    // Publishers/Subscribers
    ros::Publisher pwm_publisher;
    ros::Subscriber telemetry_sub;
    ros::Subscriber detection_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber heading_sub;
    
    custom_msgs::commands cmd_msg_;
    
    bool                    software_arm_flag = false;
    custom_msgs::commands   cmd_pwm;

    //PID errors
    double      forward_error, forward_setpoint;
    double      lateral_error, lateral_setpoint;
    double      depth_error, depth_setpoint;
    int         yaw_error, yaw_setpoint;




    GateNavigator() : 
        depth_pid_obj(), yaw_pid_obj(), lateral_pid_obj(), forward_pid_obj(),
        image_center_x(320)  // Assuming 640x480 resolution
    {
        // Initialize PID parameters
        depth_pid_obj.kp = 5.5;    depth_pid_obj.ki = 0.03;    depth_pid_obj.kd = 31.5;  depth_pid_obj.base_offset = 1580;   depth_setpoint = 1069;
        yaw_pid_obj.kp = 3.18;      yaw_pid_obj.ki = 0.01;      yaw_pid_obj.kd = 7.2;      yaw_pid_obj.base_offset = 1500;     yaw_setpoint = 280;
        lateral_pid_obj.kp = -0.5;  lateral_pid_obj.ki = -0.05; lateral_pid_obj.kd = -2.0;  lateral_pid_obj.base_offset = 1500; 
        forward_pid_obj.kp = -0.8;  forward_pid_obj.ki = -0.1;  forward_pid_obj.kd = -4.0;  forward_pid_obj.base_offset = 1500;

        // Setup communications
        pwm_publisher = nh_.advertise<custom_msgs::commands>("/master/commands", 1);
        telemetry_sub = nh_.subscribe("/master/telemetry", 1, &GateNavigator::telemetryCallback, this);
        detection_sub = nh_.subscribe("detectnet/detections", 1, &GateNavigator::detectionCallback, this);
        pose_sub = nh_.subscribe("gate_pose", 1, &GateNavigator::poseCallback, this);
        heading_sub = nh_.subscribe("/master/heading", 1, &GateNavigator::headingCallback, this);
    }


    void headingCallback(const std_msgs::Float32::ConstPtr &msg) {
       	current_yaw = msg->data;  
        yaw_error = fmod((yaw_setpoint - current_yaw + 180), 360)-180;
    }

    void telemetryCallback(const custom_msgs::telemetry::ConstPtr& msg) {
        double current_depth = msg->external_pressure;
        depth_error = depth_setpoint - current_depth;
        // int current_yaw = msg->heading;  
        // yaw_error = fmod((yaw_setpoint - current_yaw + 180), 360)-180;
    }

    void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& msg) {
        if(!msg->detections.empty()) {
            double gate_center_x = msg->detections[0].bbox.center.x;
            double lateral_error = image_center_x - gate_center_x;
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Update forward PID based on distance to gate
        double distance = msg->pose.position.x;  // Assuming x is forward distance
    }


};

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "gate_navigator");
    GateNavigator navigator;
    ros::spinOnce();
    navigator.software_arm_flag = true;
    ros::Time init_time = ros::Time::now();
    navigator.yaw_pid_obj.emptyError();
    while(ros::ok()) {
          navigator.cmd_pwm.mode       = "STABILIZE";
          ros::Time time_now = ros::Time::now();
          navigator.cmd_pwm.arm        = true;
	  if((time_now - init_time).toSec() > 5.0){
            if (navigator.software_arm_flag == true) {
		//std::cout<<navigator.yaw_error<<"\n";
                float pid_depth = navigator.depth_pid_obj.pid_control(
                    navigator.depth_error, (time_now - init_time).toSec(), false);
                
                float pid_yaw = navigator.yaw_pid_obj.pid_control(
                    navigator.yaw_error, (time_now - init_time).toSec(), false);
		//std::cout<<pid_yaw<<"\n";
                float pid_lateral = navigator.lateral_pid_obj.pid_control(
                    navigator.lateral_error, (time_now - init_time).toSec(), false);
                if ((time_now - init_time).toSec() < 10.0 ) {
                //std::cout <<" "<<pid_depth<<" "<<pid_yaw<<"\n";  // ((time_now - start_routine).toSec()) << "\n";
                navigator.cmd_pwm.forward = 1500;
                navigator.cmd_pwm.lateral = 1500;
                navigator.cmd_pwm.thrust  = pid_depth;
                navigator.cmd_pwm.yaw     = pid_yaw;
                std::cout << "sinking";
            
		}else if ((time_now - init_time).toSec() < 20.0) {
		navigator.cmd_pwm.forward = 1800;
		navigator.cmd_pwm.lateral = 1500;
		navigator.cmd_pwm.thrust  = pid_depth;
		navigator.cmd_pwm.yaw     = pid_yaw;
		std::cout << "seksi time"<<"\n";
		}
	    }else {
            navigator.depth_pid_obj.emptyError();
	    navigator.yaw_pid_obj.emptyError();
            navigator.cmd_pwm.arm     = false;
            navigator.cmd_pwm.mode    = "STABILIZE";
            navigator.cmd_pwm.forward = 1500;
            navigator.cmd_pwm.lateral = 1500;
            navigator.cmd_pwm.thrust  = 1500;
            navigator.cmd_pwm.yaw     = 1500;
        }
        navigator.pwm_publisher.publish(navigator.cmd_pwm);
       }else{	
       	  navigator.yaw_setpoint = navigator.current_yaw; 
       }
       ros::spinOnce();
    }
    return 0;
}
