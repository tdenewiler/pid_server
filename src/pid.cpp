#include <ros/ros.h>
#include <pid_server/PID.h>

bool computePIDOut(pid_server::PID::Request &req, pid_server::PID::Response &res)
{
  // Declare variables.
  float error = req.target_val - req.current_val;
  float integral_term = req.previous_integrator_val + error * req.dt;

  // Apply integrator limits.
  if (integral_term < req.integral_term_min)
  {
    integral_term = req.integral_term_min;
  }
  if (integral_term > req.integral_term_max)
  {
    integral_term = req.integral_term_max;
  }

  // Compute the output.
  res.output = req.kp * error + req.ki * integral_term + req.kd * (error - req.previous_error) / req.dt;
  res.current_integrator_val = integral_term;
  res.current_error = error;

  ROS_DEBUG("Request contains current_state = %f, target_state = %f, Kp = %f", req.current_val, req.target_val, req.kp);
  ROS_DEBUG("Integrator limits with min = %f, max = %f", req.integral_term_min, req.integral_term_max);
  ROS_DEBUG("Sending back control input -- output = %f", res.output);

  return true;
}

int main(int argc, char **argv)
{
  // Start ROS.
  ros::init(argc, argv, "pidserver");
  ros::NodeHandle n;

  // Start PID service.
  ros::ServiceServer service = n.advertiseService("pid", computePIDOut);
  ros::spin();

  return 0;
}
