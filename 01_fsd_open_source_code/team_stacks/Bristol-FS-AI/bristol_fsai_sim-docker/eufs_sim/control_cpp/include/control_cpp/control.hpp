#ifndef MY_PACKAGE_CONTROL_HPP
#define MY_PACKAGE_CONTROL_HPP

#include "eufs_msgs/msg/can_state.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <map>
#include <functional>
#include <random>
#include <cmath>
#include <vector>
#include "control_cpp/cone.hpp"
#include "control_cpp/cone_colors.hpp"
#include "spline.h"
#include <mutex>
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
// CSV logging
#include <fstream>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Enum for demo mission phases
enum DemoMissionPhase {
  DEMO_STEERING_SWEEP,
  DEMO_FIRST_ACCEL,
  DEMO_STOPPING,
  DEMO_SECOND_ACCEL,
  DEMO_EBS_AND_COMPLETE,
};

// Enum for static inspection A phases
enum InspectionAPhase {
  A_STEERING_SWEEP,
  A_RPM_RAMP_UP,
  A_RPM_HOLD_MAX,
  A_RPM_RAMP_DOWN,
  A_COMPLETED
};

// Enum for static inspection B phases
enum InspectionBPhase {
  B_RPM_RAMP_UP,
  B_RPM_HOLD,
  B_EBS_TRIGGER
};

class Control : public rclcpp::Node {
public:
  Control();
  ~Control();

private:
  // function prototypes
  void createCone(float initialx, float initialy, int colour);
  void updateParticleSet(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void processCones(const std::vector<eufs_msgs::msg::ConeWithCovariance>& detected_cones, int colour);
  void getTrackCones(std::vector<Cone*>& blue_cones, std::vector<Cone*>& yellow_cones, std::vector<Cone*>& orange_cones);
  void generateSpline();
  double find_t_for_lookahead(const tk::spline& spline_x, const tk::spline& spline_y, double max_t);
  void updateOdomFrame();

  // mission state management
  void resetMissionState();
  bool waitForDrivingState();
  void sendMissionCompletion(const std::string& mission_name);

  // callbacks
  void stateCallback(const eufs_msgs::msg::CanState::SharedPtr msg);
  void coneCallback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void odometryCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void wheelSpeedsAndSteeringCallback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg);
  void publishCommand();

  // publisher/subscriber definitions
  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr state_sub;
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr car_sub;
  rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_speeds_sub;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_completion_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cone_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr midpoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr spline_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ebs_client;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr stop_zone_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr driving_flag_pub;

  // Car parameters
  // CONSTANTS
  const float wheelbase_ = 1.53; // meters 
  const float max_accel_ = 4.5; // meters/s^2 (approx from values in ros_can.hpp)
  const float max_dec_ = 10.0; // meters/s^2 (approx from values in ros_can.hpp)
  const float wheel_radius_ = 0.253; // Radius of DDT car wheels (approx from values in ros_can.hpp)
  const double max_steering_angle_ = 24.0; // degrees, sweep from +max to -max
  const double brake_acceleration_ = -5.0; // meters/s^2, deceleration to stop
  
  // Time variables
  float deltaT = 0.0;
  rclcpp::Time last_command_time_;
  // State variables
  float velx = 0.0;
  float angularz = 0.0;
  float vely = 0.0;
  double current_steering_angle_rad_ = 0.0;
  double current_rpm_ = 0.0;
  double current_velocity_ = 0.0;
  // Command variables
  double steering = 0.0;
  double acceleration = 0.0;

  // cone locations
  int32_t blue_x, blue_y, yellow_x, yellow_y;

  //algorithm instansiation
  ackermann_msgs::msg::AckermannDriveStamped mission_acceleration();
  ackermann_msgs::msg::AckermannDriveStamped mission_skidpad();
  ackermann_msgs::msg::AckermannDriveStamped mission_autocross();
  ackermann_msgs::msg::AckermannDriveStamped mission_track_drive();
  ackermann_msgs::msg::AckermannDriveStamped static_inspection_a();
  ackermann_msgs::msg::AckermannDriveStamped static_inspection_b();
  ackermann_msgs::msg::AckermannDriveStamped autonomous_demo_mission();
  ackermann_msgs::msg::AckermannDriveStamped system_characterization_mission();
  // Driving functions
  ackermann_msgs::msg::AckermannDriveStamped decelerate_and_complete(const std::string& mission_name, double brake_accel = 0.0);
  ackermann_msgs::msg::AckermannDriveStamped step_input_characterization(double target_steering_angle_deg, double target_rpm, double step_duration, double hold_time, int char_num_repetitions_);

  // AMI state
  std::map<uint16_t, std::function<ackermann_msgs::msg::AckermannDriveStamped()>> algorithm_states;
  uint16_t current_mission_state;
  uint16_t current_as_state;

  // Mission state variables (used by all missions)
  bool driving_flag_ = false;
  rclcpp::Time driving_flag_start_time_; // When the mission should actually start executing (5 seconds after mission is set)
  bool mission_completion_sent_ = false;
  const double min_velocity_to_complete_mission_ = 0.1; // meters/s, minimum velocity to complete a mission

  // Inspection Mission state variables
  InspectionAPhase inspection_a_phase_ = A_STEERING_SWEEP;
  rclcpp::Time inspection_a_phase_start_time_tracker_;
  bool inspection_ebs_triggered_ = false;
  double inspection_target_rpm_ = 0.0;
  // CONSTANTS
  const double inspection_rpm_kp = 0.05; // proportional gain for RPM control (TUNED IN SIM)
  const double inspection_rpm_ki = 0.001; // integral gain for RPM control (TUNED IN SIM)
  const double inspection_rpm_kd = 0.0; // derivative gain for RPM control (TUNED IN SIM)
  const double inspection_steering_max_angle_ = 24.0; // degrees, sweep from +max to -max
  const double inspection_a_max_rpm_ = 200.0; // Max RPM for static inspection A
  // Helper for steering sweep math (returns {target_angle_deg, sweep_done})
  std::pair<double, bool> compute_steering_sweep(
      double elapsed_time,
      double max_angle_deg,
      double sweep_duration) const;
  // CONSTANTS
  const double inspection_steering_sweep_duration_ = 20.0; // seconds for complete sweep from +max to -max
  
  // Inspection B state variables
  InspectionBPhase inspection_b_phase_ = B_RPM_RAMP_UP;
  rclcpp::Time inspection_b_phase_start_time_tracker_;
  bool inspection_b_ebs_triggered_ = false;
  double inspection_b_target_rpm_ = 0.0;
  // PID state for RPM control
  double rpm_integral_error_ = 0.0;
  double rpm_prev_error_ = 0.0;
  // CONSTANTS
  const double inspection_b_max_rpm_ = 50.0; // Max RPM for static inspection B
  
  // Autonomous demo mission state variables
  DemoMissionPhase demo_mission_phase_ = DEMO_STEERING_SWEEP;
  double demo_mission_phase_start_distance_ = 0.0;
  double total_distance_travelled_ = 0.0;
  bool demo_mission_ebs_triggered_ = false;
  double auto_demo_target_velocity_ = 15.0 / 3.6; // 15 kph

  // Particle filter
  std::vector<Cone> coneList;
  // CONSTANTS
  const double max_dead_time_ = 1.0; // max time in seconds before cone depreciated
  const double min_cone_distance_from_car_ = 3.0; // min distance from car for a cone to be created
  const float max_cone_distance_from_car_ = 15.0;   // max distance from car for a cone to be considered
  const double cone_association_distance_ = 1.58; // absolute distance threshold for associating cones

  // Planning and Control Algorithms
  ackermann_msgs::msg::AckermannDriveStamped midpoint_planning_control(double target_velocity, double min_cone_distance_from_car_for_midpoint);
  ackermann_msgs::msg::AckermannDriveStamped cubic_spline_planning_control(double target_velocity);

  // PID steering control
  double pid_steering_control(double error, double kp, double ki, double kd, double tau);
  double pid_steering_prev_error_ = 0.0;
  double pid_steering_prev_steering_ = 0.0;
  double pid_steering_integral_error_ = 0.0;
  // CONSTANTS
  const double pid_steering_kp = 1.03;
  const double pid_steering_kd = 0.0;
  const double pid_steering_ki = 0.0;
  const double pid_steering_tau = 0.9;
  const double pid_steering_minimum_sensitivity = 0.05; // minimum error to consider for PID steering control

  // PID acceleration control
  double pid_accel_control(double target_velocity, double kp, double ki, double kd);
  double pid_accel_integral_error_ = 0.0;
  double pid_accel_prev_error_ = 0.0;
  // CONSTANTS
  const double pid_accel_kp = 8.8;
  const double pid_accel_ki = 0.0;
  const double pid_accel_kd = 0.0;

  // PID RPM control
  double pid_rpm_control(double target_rpm, double kp, double ki, double kd);

  // Stopping logic variables and functions
  bool lapsCompletedFlag(int noLaps);
  bool orange_cones_visible = false;
  int finish_line_counter_ = 0;
  int laps_completed_ = 0;
  bool laps_completed_logged_ = false;
  // Stopping logic variables (Odometry-based)
  bool lapsCompletedFlagOdom(int noLaps);
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_theta_ = 0.0;
  double lap_start_x_ = 0.0;
  double lap_start_y_ = 0.0;
  bool lap_started_ = false;
  double dist_travelled_since_lap_start_ = 0.0;
  // TF broadcaster for odom -> base_footprint transform (dead reckoning)
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // CONSTANTS
  const double MIN_LAP_DISTANCE = 10.0; // meters, min distance before a lap can be completed.
  const double LAP_COMPLETION_RADIUS = 7.5; // meters, radius to consider lap complete.
  
  // midpoint ordering
  std::vector<std::pair<double,double>> sortMidpoints(const std::vector<std::pair<double,double>>& midpoints);
  

  // Spline variables
  tk::spline spline_x_;
  tk::spline spline_y_;
  bool spline_valid_ = false;
  double spline_max_t_ = 0.0;
  std::mutex spline_mutex_;
  std::vector<geometry_msgs::msg::Point> spline_points_;
  geometry_msgs::msg::Point spline_lookahead_point_;
  std::vector<std::pair<double, double>> spline_midpoints_; // Store midpoints for visualization
  // Frame-to-frame consistency tracking
  std::vector<std::pair<size_t, size_t>> previous_cone_pairs_; // Store (blue_idx, yellow_idx) pairs from previous frame
  std::vector<std::pair<double, double>> previous_midpoints_; // Store previous frame's midpoints
  // CONSTANTS
  const double spline_search_step_ = 0.01; // Step size for spline parameter search (accuracy/performance trade-off)
  const double lookahead_distance_ = 2.5;
  const float max_cone_pairing_distance_ = 6.0;   // max distance between a blue and yellow cone to be considered a pair
  const float min_cone_pairing_distance_ = 1.0;   // minimum distance between blue and yellow cones to prevent same-side pairing

  // #######################################################
  // Debug and visualization
  // #######################################################

  bool bristol_fsai_debug_ = false;
  // cone visualisation publisher
  visualization_msgs::msg::MarkerArray conePubList = visualization_msgs::msg::MarkerArray();
  // midpoint marker
  visualization_msgs::msg::Marker midpoint_marker_;
  bool midpoint_marker_initialized_ = false;
  // midpoint locations for visualization
  float mid_x = 0.0, mid_y = 0.0;
  bool valid_midpoint = false;
  // State for dynamic visualization lifetime
  rclcpp::Time last_cone_callback_time_;
  double last_cb_dt_ = 0.1; // Last measured time delta between cone callbacks

  // Stop zone visualization
  visualization_msgs::msg::Marker stop_zone_marker_;
  bool stop_zone_marker_initialized_ = false;
  void publishStopZoneVisualization();
  
  // visualisation functions
  void publishPFConeVisualization();
  void publishMidpointVisualization();
  void publishSplineTrajectory();

  // Logging for steering sweep
  std::ofstream log_file_;
  bool log_initialized_ = false;
  bool csv_headers_written_ = false; // Track if headers have been written for current CSV
  // CSV logging function
  void logDatatoCSV(const std::string& filename, const std::vector<std::string>& headers, const std::vector<double>& values);
  
  // System characterization state variables
  enum CharacterizationPhase {
    CHAR_STEERING_STEP,
    CHAR_RPM_STEP,
    CHAR_COMPLETED
  };
  enum StepPhase {
    STEP_WAIT_ZERO,
    STEP_APPLY_TARGET,
    STEP_HOLD_TARGET,
    STEP_RETURN_ZERO
  };
  CharacterizationPhase char_phase_ = CHAR_STEERING_STEP;
  StepPhase step_phase_ = STEP_WAIT_ZERO;
  rclcpp::Time char_phase_start_time_;
  rclcpp::Time step_phase_start_time_;
  bool char_steering_step_complete_ = false;
  bool char_rpm_step_complete_ = false;
  double char_target_steering_ = 0.0;
  double char_target_rpm_ = 0.0;
  int char_current_repetition_ = 0;
  bool char_phase_started_ = false;
};

#endif