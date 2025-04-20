#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>
#include <vector>
TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));
      
    steering_angle_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));
    
    imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::imu_callback, this, std::placeholders::_1));

    lap_time_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::lap_time_callback, this, std::placeholders::_1));
    

    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
    rr_slip_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    rl_slip_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    fl_slip_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
    fr_slip_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    imu_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter",qos_profile);
    lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);

}


void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wspeed_msg) {
  //save and convert the wheel speeds to m/s
    vw_rr = wspeed_msg->rear_right * 1000.0 / 3600.0;
    vw_rl = wspeed_msg->rear_left * 1000.0 / 3600.0;
    vw_fr = wspeed_msg->front_right * 1000.0 / 3600.0;
    vw_fl = wspeed_msg->front_left * 1000.0 / 3600.0;
}

void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
  //get the steering wheel angle and convert to radians
    steering_angle = (steering_msg->primary_steering_angle_fbk / 15.0) * M_PI  / 180.0;
}


void TakeHome::imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr imu_msg) {
    rclcpp::Time stamp  = imu_msg->header.stamp;
    double current = stamp.seconds();
    //fully clearing the timestamps if the bag is reset for any reason. 
    if (!times.empty() && (current - times.back()) < -1.0) {
        times.clear();
    }
    //erase times outside of the 1 second window.
    times.push_back(current);
    while ((current - times.front()) > 1.0 && !times.empty()) {
      times.erase(times.begin());
    }
    //calculate and publish the jitter using the variance equation if there are at least two timestamps to use in the array.
    if (times.size() >= 2) {
      double sum = 0;
      double mean = 0;
      double variance = 0;
      for (size_t i = 1; i < times.size(); i++) {
        sum += times[i] - times[i-1];
      }
      mean = sum/(times.size() - 1);

      for (size_t i = 1; i < times.size(); i++) {
        double x = times[i] - times[i-1];
        variance += (x - mean) * (x - mean);
      }
      variance = variance / (times.size() -1);

      std_msgs::msg::Float32 jitter_msg;
      jitter_msg.data = variance;
      imu_publisher_->publish(jitter_msg);
    }
}

void TakeHome::lap_time_callback(std_msgs::msg::Float32::ConstSharedPtr dist_msg) {
  float distance = dist_msg->data;
  rclcpp::Time timestamp = this->now();
  double current_time = timestamp.seconds();
  if (first_lap) {
    lap_start = current_time;
    first_lap = false;
  }
  //Check to see if curvilinear distance was reset. 
  if (distance == 0.0 && (current_time-lap_start) >= 1) {
    double lap_time = (current_time - lap_start);
    std_msgs::msg::Float32 lap_msg;
    lap_msg.data = lap_time;
    lap_time_publisher_->publish(lap_msg);
    lap_start = current_time;
  }
}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;
//calculate each wheel slip ratio
  float vx = odom_msg->twist.twist.linear.x;
  float vy = odom_msg->twist.twist.linear.y;
  float w = odom_msg->twist.twist.angular.z;

  float v_rr = vx - 0.5 * w * wr;
  float k_rr = (vw_rr-v_rr) / v_rr;

  float v_rl = vx + 0.5 * w * wr;
  float k_rl = (vw_rl-v_rl) / v_rl;

  float vx_fr  = vx - 0.5 * w * wf;
  float vy_fr  = vy + w * lf;
  float vdx_fr = cos(steering_angle)*vx_fr - sin(steering_angle)*vy_fr;
  float k_fr = (vw_fr - vdx_fr)/vdx_fr;

  float vx_fl  = vx + 0.5 * w * wf;
  float vy_fl  = vy + w * lf;
  float vdx_fl = cos(steering_angle)*vx_fl - sin(steering_angle)*vy_fl;
  float k_fl = (vw_fl - vdx_fl)/vdx_fl;

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);
  //Publish to topics for each wheel slip ratio
  std_msgs::msg::Float32 rr_msg;
  rr_msg.data = k_rr;
  rr_slip_publisher_->publish(rr_msg);

  std_msgs::msg::Float32 rl_msg;
  rl_msg.data = k_rl;
  rl_slip_publisher_->publish(rl_msg);

  std_msgs::msg::Float32 fr_msg;
  fr_msg.data = k_fr;
  fr_slip_publisher_->publish(fr_msg);

  std_msgs::msg::Float32 fl_msg;
  fl_msg.data = k_fl;
  fl_slip_publisher_->publish(fl_msg);

  
}
RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
