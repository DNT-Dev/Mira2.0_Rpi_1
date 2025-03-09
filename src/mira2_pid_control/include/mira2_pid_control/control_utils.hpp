#include <cmath>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

class PID_Controller {
public:
  float safe_pwm = 400;
  float kp, kd, ki, base_offset;
  float output_pwm;
  float pwm_prev = 0;
  float prev_d = 0;
  float pid_p, pid_i, pid_d = 0;
  std::vector<float> error_vector;
  float pid_control(float error, float dtime, bool switch_polarity) {
    //std::cout<<"\n"<<dtime<<" "<<error<<"\n";
    time.push_back(dtime);
    error_vector.push_back(error);
    pid_p = (kp * error);
    pid_i = (integrate(error_vector, time));
    pid_d = 0;
    try {
      if (error_vector[error_vector.size() - 1] ==
          error_vector[error_vector.size() - 2]) {
        pid_d = prev_d;
      } else {
        pid_d = (kd * (error_vector[error_vector.size() - 1] -
                       error_vector[error_vector.size() - 2]));
      }
    } catch (...) {
      pid_d = 0;
    }

    pid_i = ki * pid_i;
    //std::cout<<pid_p<<" "<<pid_d<<" "<<pid_i<<"\n";
    output_pwm = pid_p + pid_d + pid_i;
    output_pwm = base_offset + output_pwm;
    if (output_pwm > (1500+safe_pwm)) {
      output_pwm = 1500 + safe_pwm;
    } else if (output_pwm < (1500-safe_pwm)) {
      output_pwm = 1500 - safe_pwm;
    }
    prev_d = pid_d;
    pwm_prev = output_pwm;
    // std::cout << "Error stack: " << error_vector.size() << std::endl;
    // std::cout << output_pwm << "\n";
    return output_pwm;
  }
  float hold() { return 1500.00; }
  void emptyError() {
    // for (int i=0; i<error_vector.size(); i++) {
    while (error_vector.size() != 0) {
      error_vector.pop_back();
      time.pop_back();
    }
    // std::cout << "Error stack: " << error_vector.size() << std::endl;
  }

private:
  std::vector<float> time;
  float integrate(std::vector<float> error, std::vector<float> time) {
    float area = 0;
    // try {
    //     for (int i = time.size()-2000000; i < time.size(); i++) {
    //         area += (time[i] - time[i-1]) * (error[i] + error[i-1]) / 2;
    //     }
    // }
    // catch(...) {
    try {
      for (int i = 1; i < time.size(); i++) {
        area += (time[i] - time[i - 1]) * (error[i] + error[i - 1]) / 2;
      }
    } catch (...) {
    }
    // }

    // std::cout << area << std::endl;

    return area;
  }
};
