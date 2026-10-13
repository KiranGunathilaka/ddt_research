#include "control_cpp/control.hpp"
using CanState = eufs_msgs::msg::CanState;

Control::Control() : Node("control") {
  RCLCPP_INFO(this->get_logger(), "Control node initialized!");

  // Declare and get visualization parameter
  this->declare_parameter("bristol_fsai_debug", false);
  bristol_fsai_debug_ = this->get_parameter("bristol_fsai_debug").as_bool();
  RCLCPP_INFO(this->get_logger(), "bristol_fsai_debug: %s", bristol_fsai_debug_ ? "enabled" : "disabled");

  ackermann_msgs::msg::AckermannDriveStamped acceleration();
  ackermann_msgs::msg::AckermannDriveStamped skidpad();

  // CAN state subscriber
  state_sub = this->create_subscription<CanState>(
    "/ros_can/state", 10, std::bind(&Control::stateCallback, this, std::placeholders::_1)
  );

  // cone location subscriber
  cone_sub = this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
    // for perception system cone data use: "/perception/cones"
    "/cones", 10, std::bind(&Control::coneCallback, this, std::placeholders::_1)
  );

  // odometry subscriber
  car_sub = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/ros_can/twist", 10, std::bind(&Control::odometryCallback, this, std::placeholders::_1)
  );

  // wheel speeds subscriber
  wheel_speeds_sub = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
    "/ros_can/wheel_speeds", 10, std::bind(&Control::wheelSpeedsAndSteeringCallback, this, std::placeholders::_1)
  );

  // command publisher
  cmd_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
  timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Control::publishCommand, this));

  // mission completion publisher
  mission_completion_pub = this->create_publisher<std_msgs::msg::Bool>("/ros_can/mission_completed", 10);

  // EBS service client
  ebs_client = this->create_client<std_srvs::srv::Trigger>("/ros_can/ebs");

  // Driver flag publisher
  driving_flag_pub = this->create_publisher<std_msgs::msg::Bool>("/state_machine/driving_flag", 10);

  // Cones (after particle filter)
  cone_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pf_cones", 10);

  // DEBUGGING
  spline_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/spline_trajectory", 10);

  // DEBUGGING
  stop_zone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/stop_zone_marker", 10);

  // publisher of pf cone locations (only create if visualization is enabled)
  if (bristol_fsai_debug_) {
    midpoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/midpoint_marker", 10);
    // Initialize last cone callback time for cone visualization
    last_cone_callback_time_ = this->now();
  }

  // tf broadcaster for odom -> base_footprint transform (dead reckoning)
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  //algorithm switching
  algorithm_states[CanState::AMI_ACCELERATION] = std::bind(&Control::mission_acceleration, this);
  algorithm_states[CanState::AMI_SKIDPAD] = std::bind(&Control::mission_skidpad, this);
  algorithm_states[CanState::AMI_AUTOCROSS] = std::bind(&Control::mission_autocross, this);
  algorithm_states[CanState::AMI_TRACK_DRIVE] = std::bind(&Control::mission_track_drive, this);
  
  // algorithm_states[CanState::AMI_DDT_INSPECTION_A] = std::bind(&Control::static_inspection_a, this);
  // TEMPORARY REMAPPING
  algorithm_states[CanState::AMI_DDT_INSPECTION_A] = std::bind(&Control::system_characterization_mission, this);
  
  algorithm_states[CanState::AMI_DDT_INSPECTION_B] = std::bind(&Control::static_inspection_b, this);
  algorithm_states[CanState::AMI_AUTONOMOUS_DEMO] = std::bind(&Control::autonomous_demo_mission, this);

}

// Destructor to close the log file
Control::~Control() {
  if (log_file_.is_open()) {
    log_file_.close();
  }
}


// #######################################################
// SUBSCRIBER CALLBACKS
// #######################################################


// odometry callback function
void Control::odometryCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
  velx = msg->twist.twist.linear.x;
  angularz = msg->twist.twist.angular.z;

  current_velocity_ = std::sqrt(velx * velx + vely * vely); // only use x velocity for now

  for (auto& cone : coneList) {
    cone.odometryUpdate(this->velx, this->angularz, msg->header.stamp.nanosec);
  }
}

void Control::wheelSpeedsAndSteeringCallback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg) {
  // Update current RPM based on actual wheel speed feedback
  // Use average of rear wheels for RPM tracking (typical for rear-wheel drive vehicles)
  double rear_left_rpm = msg->speeds.lb_speed;
  double rear_right_rpm = msg->speeds.rb_speed;
  current_rpm_ = (rear_left_rpm + rear_right_rpm) / 2.0;

  // Update current steering angle from feedback
  current_steering_angle_rad_ = msg->speeds.steering;
}


// CAN state callback function
void Control::stateCallback(const CanState::SharedPtr msg) {
  // Set mission state to the new state
  if (current_mission_state != msg->ami_state) {
    resetMissionState();
  }
  current_mission_state = msg->ami_state;

  // Set AS state to new state
  if (driving_flag_ == false && msg->as_state == eufs_msgs::msg::CanState::AS_DRIVING) {
    driving_flag_ = true;
    driving_flag_start_time_ = this->now(); // Set time only when driving starts
  } else if (driving_flag_ == true && msg->as_state != eufs_msgs::msg::CanState::AS_DRIVING) {
    driving_flag_ = false;
  }
  current_as_state = msg->as_state;

  auto message = std_msgs::msg::Bool();
  message.data = driving_flag_;
  driving_flag_pub->publish(message);

}


void Control::resetMissionState() {
  mission_completion_sent_ = false;

  // Reset inspection-specific state variables
  current_rpm_ = 0.0;
  inspection_target_rpm_ = 0.0;
  inspection_ebs_triggered_ = false;
  inspection_a_phase_ = A_STEERING_SWEEP;
  inspection_a_phase_start_time_tracker_ = this->now();

  // Reset inspection B state variables
  inspection_b_phase_ = B_RPM_RAMP_UP;
  inspection_b_phase_start_time_tracker_ = this->now();
  inspection_b_ebs_triggered_ = false;
  inspection_b_target_rpm_ = 0.0;

  // Reset PID state for RPM control
  rpm_integral_error_ = 0.0;
  rpm_prev_error_ = 0.0;

  // Reset PID steering state
  pid_steering_integral_error_ = 0.0;

  // Reset autonomous demo mission state
  demo_mission_phase_ = DEMO_STEERING_SWEEP;
  demo_mission_phase_start_distance_ = 0.0;
  total_distance_travelled_ = 0.0;
  demo_mission_ebs_triggered_ = false;

  // Reset system characterization state
  char_phase_ = CHAR_STEERING_STEP;
  step_phase_ = STEP_WAIT_ZERO;
  char_steering_step_complete_ = false;
  char_rpm_step_complete_ = false;
  char_target_steering_ = 0.0;
  char_target_rpm_ = 0.0;
  char_current_repetition_ = 0;

  // Reset odometry-based lap tracking state
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_theta_ = 0.0;
  lap_start_x_ = 0.0;
  lap_start_y_ = 0.0;
  lap_started_ = false;
  laps_completed_ = 0;
  dist_travelled_since_lap_start_ = 0.0;
}


void Control::sendMissionCompletion(const std::string& mission_name) {
  if (!mission_completion_sent_) {
    auto completion_msg = std_msgs::msg::Bool();
    completion_msg.data = true;
    mission_completion_pub->publish(completion_msg);
    mission_completion_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "%s mission completion signal sent", mission_name.c_str());
  }
}


// Helper function to process cones of a given color
void Control::processCones(
    const std::vector<eufs_msgs::msg::ConeWithCovariance>& detected_cones,
    int colour) 
{
  for (const auto& detected_cone : detected_cones) {
    // Check if cone is within accepted distance from car
    double distance_from_car = std::hypot(detected_cone.point.x, detected_cone.point.y);
    if (distance_from_car > max_cone_distance_from_car_) {
      continue; // Skip this cone if it's too far away
    }
    
    bool consumed = false;
    for (auto& cone : coneList) {
      if (cone.deadTimer < max_dead_time_ &&
          std::hypot(detected_cone.point.x - cone.x, detected_cone.point.y - cone.y) < cone_association_distance_) {
        // Update the particle set
        consumed = true;
        cone.perceptionUpdate(detected_cone.point.x, detected_cone.point.y);
        cone.updateColour(colour);
        cone.deadTimer = 0.0;
        break; // break out of the loop after updating the cone
      }
    }
    // Create a new cone if no matching cone found and not adjacent to vehicle
    if (!consumed && detected_cone.point.x > min_cone_distance_from_car_) {
      createCone(detected_cone.point.x, detected_cone.point.y, colour);
    }
  }
}


void Control::coneCallback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg) {
  if (bristol_fsai_debug_) {
    // Dynamically calculate time since last callback for marker lifetime
    auto now = this->now();
    double dt = (now - last_cone_callback_time_).seconds();
    // Sanity check dt and update if it's a reasonable value
    if (dt > 0.001 && dt < 1.0) {
      last_cb_dt_ = dt;
    }
    last_cone_callback_time_ = now;
  }

  // Increment deadTimer for all cones
  for (auto& cone : coneList) {
    cone.deadTimer += last_cb_dt_;
  }

  // Remove cones not updated in max_dead_time_ seconds OR too far from vehicle
  coneList.erase(
    std::remove_if(coneList.begin(), coneList.end(),
                   [this](Cone& cone) { 
                     double distance = std::hypot(cone.x, cone.y);
                     return cone.deadTimer >= max_dead_time_ || distance > max_cone_distance_from_car_;
                   }),
    coneList.end()
  );

  // Process blue and yellow cones
  processCones(msg->big_orange_cones, ORANGE);
  processCones(msg->blue_cones, BLUE);
  processCones(msg->yellow_cones, YELLOW);

  // Generate the spline from the updated cone positions
  generateSpline();

  // Publish the visualization of cones
  publishPFConeVisualization();
}


void Control::generateSpline() {
  // Protect the spline generation with a mutex to avoid race conditions between this function and cubic_spline_planning_control
  std::lock_guard<std::mutex> lock(spline_mutex_);

  // Clear previous spline visualization data
  if (bristol_fsai_debug_) {
    spline_points_.clear();
    spline_midpoints_.clear();
  }

  std::vector<Cone*> blue_cones_ptr;
  std::vector<Cone*> yellow_cones_ptr;
  std::vector<Cone*> orange_cones_ptr;
  getTrackCones(blue_cones_ptr, yellow_cones_ptr, orange_cones_ptr);

  if (blue_cones_ptr.empty() || yellow_cones_ptr.empty()) {
    spline_valid_ = false;
    return;
  }

  std::vector<std::pair<double, double>> midpoints;
  std::vector<std::pair<size_t, size_t>> current_cone_pairs;

  // Create all possible valid pairings between blue and yellow cones
  for (size_t blue_idx = 0; blue_idx < blue_cones_ptr.size(); ++blue_idx) {
    const auto& blue_cone = blue_cones_ptr[blue_idx];
    
    for (size_t yellow_idx = 0; yellow_idx < yellow_cones_ptr.size(); ++yellow_idx) {
      const auto& yellow_cone = yellow_cones_ptr[yellow_idx];
      
      double dist = std::hypot(blue_cone->x - yellow_cone->x, blue_cone->y - yellow_cone->y);
      
      // Create midpoint if distance is within valid range
      if (dist < max_cone_pairing_distance_ && dist > min_cone_pairing_distance_) {
        midpoints.emplace_back((blue_cone->x + yellow_cone->x) / 2.0, (blue_cone->y + yellow_cone->y) / 2.0);
        current_cone_pairs.emplace_back(blue_idx, yellow_idx);
        
        // Store midpoints for visualization
        if (bristol_fsai_debug_) {
          spline_midpoints_.emplace_back((blue_cone->x + yellow_cone->x) / 2.0, (blue_cone->y + yellow_cone->y) / 2.0);
        }
      }
    }
  }

  // if orange cones are visible, reset the finish line counter
  if (!orange_cones_ptr.empty()) {
    finish_line_counter_ = 0; 
    orange_cones_visible = true;
  } 
  else finish_line_counter_++;

  // Sort midpoints by distance from the car to order them along the track
  // std::sort(midpoints.begin(), midpoints.end(), [](const auto& a, const auto& b) {
  //   return std::hypot(a.first, a.second) < std::hypot(b.first, b.second);
  // });

  midpoints = sortMidpoints(midpoints);

  // Store current pairs and midpoints for next frame
  previous_cone_pairs_ = current_cone_pairs;
  previous_midpoints_ = midpoints;

  std::vector<double> x_mid, y_mid;
  // Insert the car's current position (origin) at the start of the spline
  x_mid.push_back(0.0);
  y_mid.push_back(0.0);

  for (const auto& p : midpoints) {
    x_mid.push_back(p.first);
    y_mid.push_back(p.second);
  }

  // Generate cubic spline if there are enough points
  if (x_mid.size() >= 3) {
    std::vector<double> t_coords;
    t_coords.push_back(0.0);
    for (long unsigned int i = 1; i < x_mid.size(); ++i) {
      double dx = x_mid[i] - x_mid[i - 1];
      double dy = y_mid[i] - y_mid[i - 1];
      double dist = std::sqrt(dx * dx + dy * dy);
      t_coords.push_back(t_coords.back() + dist);
    }

    spline_x_.set_points(t_coords, x_mid);
    spline_y_.set_points(t_coords, y_mid);
    spline_max_t_ = t_coords.back();
    spline_valid_ = true;

  } else {
    spline_valid_ = false;
    // RCLCPP_INFO(this->get_logger(), "Not enough points to generate spline");
  }
}


// function to initialise a new particle set according to a gaussian
void Control::createCone(float initialx, float initialy, int colour) {
  std::random_device rd;
  std::mt19937 gen(rd());
  // initial variance of 1.0 as values will be very diffuse
  std::normal_distribution<float> distribution(0.0, 1.0);

  coneList.emplace_back(initialx, initialy, colour);
  coneList[coneList.size() - 1].createParticleSet();
}

std::vector<std::pair<double,double>> Control::sortMidpoints(const std::vector<std::pair<double,double>>& midpoints) {
    if (midpoints.empty()) return {};

  
    double smallestDist = std::numeric_limits<double>::max();
    int midIndex = 0;
    for (size_t i = 0; i < midpoints.size(); i++){
      double dist = std::hypot(midpoints[i].first, midpoints[i].second);
      if (dist < smallestDist){
        smallestDist = dist;
        midIndex = i;
      }
    }

    std::vector<std::pair<double,double>> ordered;
    std::vector<bool> visited(midpoints.size(), false);

    // Start with the first point
    ordered.push_back(midpoints[midIndex]);
    visited[midIndex] = true;

    for (size_t i = 0; i < midpoints.size() - 1; ++i) {
        const std::pair<double,double>& current = ordered.back();
        size_t bestIndex = 0;
        double bestDist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < midpoints.size(); ++j) {
            if (!visited[j]) {
                double dx = midpoints[j].first - current.first;
                double dy = midpoints[j].second - current.second;
                double dist = std::hypot(dx, dy);

                if (dist < bestDist) {
                    bestDist = dist;
                    bestIndex = j;
                }
            }
        }

        visited[bestIndex] = true;
        ordered.push_back(midpoints[bestIndex]);
    }

    return ordered;
  }

// #######################################################
// PUBLISHER CALLBACKS
// #######################################################


ackermann_msgs::msg::AckermannDriveStamped Control::mission_acceleration(void) {
  
  double target_velocity = 3.0; // meters/s
  double min_cone_distance_from_car_for_midpoint = 3.0; // meters
  double length_of_acceleration_track = 75.0; // meters
  double stopping_distance = 20.0; // (maximimum is 100m in rulebook)

  ackermann_msgs::msg::AckermannDriveStamped drive;

  drive = midpoint_planning_control(target_velocity, min_cone_distance_from_car_for_midpoint);

  if (total_distance_travelled_ > (stopping_distance + length_of_acceleration_track)) {
      drive = decelerate_and_complete("Acceleration");
  }
  
  // int noLaps = 1; // number of laps to complete
  // if (lapsCompletedFlag(noLaps)) {
  //   // wait until the car is within the maximum stopping distance
  //   double distance_to_wait_until_stop = maximum_stopping_distance * 0.5; // meters
  //   if (dist_travelled_since_lap_start_ > distance_to_wait_until_stop) {
  //     drive = decelerate_and_complete("Sprint");
  //   }
  // }

  return drive;
}

ackermann_msgs::msg::AckermannDriveStamped Control::mission_autocross(void) {

  double target_velocity = 1.0; // meters/s
  int noLaps = 1; // number of laps to complete
  double maximum_stopping_distance = 30.0; // meters

  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive = cubic_spline_planning_control(target_velocity);

  if (lapsCompletedFlagOdom(noLaps)) {
    // wait until the car is within the maximum stopping distance
    double distance_to_wait_until_stop = maximum_stopping_distance * 0.5; // meters
    if (dist_travelled_since_lap_start_ > distance_to_wait_until_stop) {
      drive = decelerate_and_complete("Sprint");
    }
  }

  return drive;
}

ackermann_msgs::msg::AckermannDriveStamped Control::mission_track_drive(void) {

  double target_velocity = 1.0; // meters/s
  int noLaps = 10; // number of laps to complete
  double maximum_stopping_distance = 30.0; // meters

  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive = cubic_spline_planning_control(target_velocity);

  if (lapsCompletedFlagOdom(noLaps)) {
    // wait until the car is within the maximum stopping distance
    double distance_to_wait_until_stop = maximum_stopping_distance * 0.5; // meters
    if (dist_travelled_since_lap_start_ > distance_to_wait_until_stop) {
      drive = decelerate_and_complete("Track Drive");
    }
  }

  return drive;
}



ackermann_msgs::msg::AckermannDriveStamped Control::mission_skidpad(void) {

  ackermann_msgs::msg::AckermannDriveStamped drive;
  RCLCPP_INFO(this->get_logger(), "[WARNING] Skidpad mission not programmed yet");
  
  return drive;
}


void Control::publishCommand() {
  // Calculate deltaT for PD controller
  auto current_time = this->now();
  if (last_command_time_.nanoseconds() > 0) {
    deltaT = (current_time - last_command_time_).seconds();
  } else {
    deltaT = 0.1; // Default to 100ms on first call
  }
  last_command_time_ = current_time;

  // Update odometry frame (by simple dead reckoning)
  updateOdomFrame();

  // check if state_sub is in AS_DRIVING
  if (driving_flag_ && current_as_state == eufs_msgs::msg::CanState::AS_DRIVING) {

    auto drive = ackermann_msgs::msg::AckermannDriveStamped();
    if (algorithm_states.count(current_mission_state)) {
      drive = algorithm_states[current_mission_state]();
    } else {
      drive.drive.speed = 0.0;
      drive.drive.acceleration = 0.0;
      drive.drive.steering_angle = 0.0;
    }
    cmd_pub->publish(drive);

  } else {
    // if not in AS_DRIVING, publish neutral commands
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = this->now();
    drive.drive.steering_angle = 0.0;
    drive.drive.steering_angle_velocity = 0.0;
    drive.drive.speed = 0.0;
    drive.drive.acceleration = 0.0;
    cmd_pub->publish(drive);
  }

  // Call the visualization functions
  publishStopZoneVisualization();
}

void Control::updateOdomFrame() {
  // Integrate odometry
  double dist_increment = velx * deltaT;
  odom_theta_ += angularz * deltaT;
  odom_x_ += dist_increment * cos(odom_theta_);
  odom_y_ += dist_increment * sin(odom_theta_);
  total_distance_travelled_ += dist_increment;

  if (lap_started_) {
      dist_travelled_since_lap_start_ += dist_increment;
  }

  // Publish odom_tmp -> base_footprint transform
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = this->now();
  odom_tf.header.frame_id = "odom_tmp";
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom_x_;
  odom_tf.transform.translation.y = odom_y_;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation.x = 0.0;
  odom_tf.transform.rotation.y = 0.0;
  odom_tf.transform.rotation.z = sin(odom_theta_ / 2.0);
  odom_tf.transform.rotation.w = cos(odom_theta_ / 2.0);
  tf_broadcaster_->sendTransform(odom_tf);
}

// #######################################################
// P&C ALGORITHMS
// #######################################################


// Helper function to decelerate and send mission completion
ackermann_msgs::msg::AckermannDriveStamped Control::decelerate_and_complete(const std::string& mission_name, double brake_accel) {
  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive.header.stamp = this->now();
  if (std::abs(current_velocity_) > min_velocity_to_complete_mission_) {
      drive.drive.steering_angle = 0.0;
      drive.drive.steering_angle_velocity = 0.0;
      drive.drive.speed = 0.0;
      if (brake_accel == 0.0) {
        drive.drive.acceleration = brake_acceleration_; // decelerate to stop
      } else {
        drive.drive.acceleration = brake_accel;
      }
  } else {
      drive.drive.steering_angle = 0.0;
      drive.drive.steering_angle_velocity = 0.0;
      drive.drive.speed = 0.0;
      drive.drive.acceleration = 0.0;
      sendMissionCompletion(mission_name);
  }
  return drive;
}

bool Control::lapsCompletedFlag(int noLaps){
  if (finish_line_counter_ > 50 && orange_cones_visible) {
    laps_completed_++; 
    orange_cones_visible = false;
    RCLCPP_INFO(this->get_logger(), "LAP COMPLETED: %d/%d (finished line counter: %d)", laps_completed_, noLaps, finish_line_counter_);
  }

  if (laps_completed_ - 1 >= noLaps) { // -1 because to ignore first lap
    if (!laps_completed_logged_) {
      RCLCPP_INFO(this->get_logger(), "ALL LAPS COMPLETED: %d/%d, sending laps completed flag", laps_completed_ - 1, noLaps);
      laps_completed_logged_ = true;
    }
    return true;
  }
  else {
    return false; 
  }
}

bool Control::lapsCompletedFlagOdom(int noLaps){
    if (!lap_started_) {
        // First time this is called for the mission, set start point
        lap_start_x_ = odom_x_;
        lap_start_y_ = odom_y_;
        lap_started_ = true;
        RCLCPP_INFO(this->get_logger(), "Lap tracking started at (%.2f, %.2f)", lap_start_x_, lap_start_y_);
        return false;
    }

    double dist_to_start = std::hypot(odom_x_ - lap_start_x_, odom_y_ - lap_start_y_);

    if (dist_travelled_since_lap_start_ > MIN_LAP_DISTANCE && dist_to_start < LAP_COMPLETION_RADIUS) {
        laps_completed_++;
        RCLCPP_INFO(this->get_logger(), "LAP COMPLETED: %d/%d", laps_completed_, noLaps);

        // Reset for next lap
        lap_start_x_ = odom_x_;
        lap_start_y_ = odom_y_;
        dist_travelled_since_lap_start_ = 0.0;
    }

    if (laps_completed_ >= noLaps) {
        if (!laps_completed_logged_) {
          RCLCPP_INFO(this->get_logger(), "ALL LAPS COMPLETED: %d/%d, returning laps completed flag", laps_completed_, noLaps);
          laps_completed_logged_ = true;
        }
        return true;
    } else {
        return false;
    }
}

void Control::getTrackCones(
  std::vector<Cone*>& blue_cones,
  std::vector<Cone*>& yellow_cones,
  std::vector<Cone*>& orange_cones)
{
    blue_cones.clear();
    yellow_cones.clear();
    orange_cones.clear();
    for (auto& cone : coneList) {
        if (cone.deadTimer < max_dead_time_) {
            int color = cone.getColour();
            if (color == ORANGE) {
                orange_cones.push_back(&cone);
            } else if (color == YELLOW) {
                yellow_cones.push_back(&cone);
            } else if (color == BLUE) {
                blue_cones.push_back(&cone);
            }
        }
    }
}


ackermann_msgs::msg::AckermannDriveStamped Control::midpoint_planning_control(double target_velocity, double min_cone_distance_from_car_for_midpoint) {
  ackermann_msgs::msg::AckermannDriveStamped drive; 
  drive.header.stamp = this->now();

  // Get acceleration from PID controller
  acceleration = pid_accel_control(target_velocity, pid_accel_kp, pid_accel_ki, pid_accel_kd);
  drive.drive.acceleration = acceleration;

  std::vector<Cone*> blue_cones, yellow_cones, orange_cones;
  getTrackCones(blue_cones, yellow_cones, orange_cones);

  Cone* closest_left = nullptr;
  Cone* closest_right = nullptr;

  double min_dist_left = 1000.0;
  for (auto& cone : yellow_cones) {
      if (cone->x > min_cone_distance_from_car_for_midpoint && cone->x < min_dist_left) {
          closest_left = cone;
          min_dist_left = cone->x;
      }
  }

  double min_dist_right = 1000.0;
  for (auto& cone : blue_cones) {
      if (cone->x > min_cone_distance_from_car_for_midpoint && cone->x < min_dist_right) {
          closest_right = cone;
          min_dist_right = cone->x;
      }
  }

  if (closest_left == nullptr || closest_right == nullptr) {
    drive.drive.speed = 0.0;
    drive.drive.steering_angle = 0.0;
    drive.drive.steering_angle_velocity = 0.0;

    RCLCPP_INFO(this->get_logger(), "No valid midpoint found");
    valid_midpoint = false;
    return drive;
  }
  else {
    mid_x = (closest_right->x + closest_left->x) / 2;
    mid_y = (closest_right->y + closest_left->y) / 2;
    valid_midpoint = true;
  }

  // Calculate error for PD controller (same as cubic_spline_planning_control)
  double distance = std::hypot(mid_x, mid_y);
  if (distance < 1e-3) distance = 1e-3;
  double angle = atan2(mid_y, mid_x);
  double error = atan((2 * wheelbase_ * sin(angle)) / distance);

  // Get steering angle from PD controller
  steering = pid_steering_control(error, pid_steering_kp, pid_steering_ki, pid_steering_kd, pid_steering_tau);

  drive.drive.speed = 0.0;
  drive.drive.acceleration = acceleration;
  drive.drive.steering_angle = steering;
  drive.drive.steering_angle_velocity = 0.0;

  // Call the visualization function
  publishMidpointVisualization();

  return drive;  
}


// Helper function to find the parameter t for a given arc length (lookahead distance) from the origin along the spline
double Control::find_t_for_lookahead(const tk::spline& spline_x, const tk::spline& spline_y, double max_t) {
    double t = 0.0;
    double accumulated = 0.0;
    double prev_x = spline_x(0.0);
    double prev_y = spline_y(0.0);

    for (t = spline_search_step_; t <= max_t; t += spline_search_step_) {
        double curr_x = spline_x(t);
        double curr_y = spline_y(t);
        accumulated += std::hypot(curr_x - prev_x, curr_y - prev_y);
        if (accumulated >= lookahead_distance_) {
            return t;
        }
        prev_x = curr_x;
        prev_y = curr_y;
    }
    return max_t; // If lookahead is longer than the spline, return the end
}


ackermann_msgs::msg::AckermannDriveStamped Control::cubic_spline_planning_control(double target_velocity) {
  if (bristol_fsai_debug_) {
    // We clear spline points here for visualization. The spline itself is generated in coneCallback.
    spline_points_.clear();
  }

  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive.header.stamp = this->now();

  // Get acceleration from PID controller
  acceleration = pid_accel_control(target_velocity, pid_accel_kp, pid_accel_ki, pid_accel_kd);
  drive.drive.acceleration = acceleration;

  double dx, dy;

  // Use the pre-computed spline (protected by mutex to avoid race conditions between coneCallback and this function)
  std::lock_guard<std::mutex> lock(spline_mutex_);

  if (spline_valid_) {
    // Ensure lookahead is a fixed distance from the car's position (origin) along the spline
    double t_lookahead = find_t_for_lookahead(spline_x_, spline_y_, spline_max_t_);
    dx = spline_x_(t_lookahead);
    dy = spline_y_(t_lookahead);
    
    if (bristol_fsai_debug_) {
      // Generate points for visualization from the spline
      for (double i = 0; i < spline_max_t_; i += 0.1) {
          geometry_msgs::msg::Point p;
          p.x = spline_x_(i);
          p.y = spline_y_(i);
          p.z = 0;
          spline_points_.push_back(p);
      }
      // Store the lookahead point for visualization
      spline_lookahead_point_.x = dx;
      spline_lookahead_point_.y = dy;
      spline_lookahead_point_.z = 0;
    }
  } else {
    // Reset PID acceleration and steering integral and previous error and steering
    pid_accel_integral_error_ = 0.0;
    pid_accel_prev_error_ = 0.0;
    pid_steering_prev_steering_ = 0.0;
    pid_steering_prev_error_ = 0.0;
    pid_steering_integral_error_ = 0.0; // Reset integral error for steering

    // Fallback if insufficient cones are available
    // Here we re-fetch cones to use the original fallback logic
    std::vector<Cone*> blue_cones_ptr, yellow_cones_ptr, orange_cones_ptr;
    getTrackCones(blue_cones_ptr, yellow_cones_ptr, orange_cones_ptr);

    if ((blue_cones_ptr.size() >= 2 && steering < 0) || (yellow_cones_ptr.size() >= 2 && steering > 0)){
      RCLCPP_INFO(this->get_logger(), "Fallback 1 triggered");
      drive.drive.speed = 1.0;
      drive.drive.steering_angle = steering * 1.5;
      drive.drive.steering_angle_velocity = 0.0;
      return drive;
    }
    // Force positive steering if only blue cones are seen
    else if (blue_cones_ptr.size() >= 2 && steering > 0){
      RCLCPP_INFO(this->get_logger(), "Fallback 2 triggered");
      drive.drive.speed = 1.0;
      drive.drive.steering_angle = -.5;
      drive.drive.steering_angle_velocity = 0.0;
      return drive;
    }
    // Force negative steering if only yellow cones are seen
    else if (yellow_cones_ptr.size() >= 2 && steering < 0){
      RCLCPP_INFO(this->get_logger(), "Fallback 3 triggered");
      drive.drive.speed = 1.0;
      drive.drive.steering_angle = .5;
      drive.drive.steering_angle_velocity = 0.0;
      return drive;
    }
    // If no fallback is triggered, just return an empty drive command
    return drive;
  }

  // Calculate error for PD controller
  double distance = std::hypot(dx, dy);
  if (distance < 1e-3) distance = 1e-3;
  double angle = atan2(dy, dx);
  double error = atan((2 * wheelbase_ * sin(angle)) / distance);
  
  // Get steering angle from PD controller
  steering = pid_steering_control(error, pid_steering_kp, pid_steering_ki, pid_steering_kd, pid_steering_tau);

  drive.drive.speed = 0.0;
  drive.drive.steering_angle = steering;
  drive.drive.steering_angle_velocity = 0.0;

  // Call the visualization function
  publishSplineTrajectory();

  return drive;
}


double Control::pid_steering_control(double error, double kp, double ki, double kd, double tau) {
  if (std::abs(error) < pid_steering_minimum_sensitivity) error = 0;

  // Proportional term
  double proportional = error * kp;
  // Derivative term
  double derivative = kd * (error - pid_steering_prev_error_) / std::max(1e-3, static_cast<double>(deltaT));
  // Integral term
  pid_steering_integral_error_ += error * deltaT;
  double integral = ki * pid_steering_integral_error_;

  pid_steering_prev_error_ = error;
  double raw_steering = proportional + integral + derivative;

  // Apply low-pass filter to reduce noise
  double filtered_steering = tau * raw_steering + (1 - tau) * pid_steering_prev_steering_;
  double clamped_steering = std::clamp(filtered_steering, -max_steering_angle_, max_steering_angle_);
  pid_steering_prev_steering_ = clamped_steering;

  return clamped_steering;
}

double Control::pid_accel_control(double target_velocity, double kp, double ki, double kd) {
  double error = target_velocity - current_velocity_;
  // Proportional term
  double proportional = kp * error;
  // Integral term
  pid_accel_integral_error_ += error * deltaT;
  double integral = ki * pid_accel_integral_error_;
  // Derivative term
  double derivative = kd * (error - pid_accel_prev_error_) / std::max(1e-3, static_cast<double>(deltaT));
  pid_accel_prev_error_ = error;
  
  // Combine terms
  acceleration = proportional + integral + derivative;
  // Clamp output
  acceleration = std::clamp(acceleration, static_cast<double>(-max_dec_), static_cast<double>(max_accel_));

  return acceleration;
}

double Control::pid_rpm_control(double target_rpm, double kp, double ki, double kd) {
  double error = target_rpm - current_rpm_;
  // Proportional term
  double proportional = kp * error;
  // Integral term
  rpm_integral_error_ += error * deltaT;
  double integral = ki * rpm_integral_error_;
  // Derivative term
  double derivative = kd * (error - rpm_prev_error_) / std::max(1e-3, static_cast<double>(deltaT));
  rpm_prev_error_ = error;
  
  // Combine terms
  acceleration = proportional + integral + derivative;
  // Clamp output
  acceleration = std::clamp(acceleration, static_cast<double>(-max_dec_), static_cast<double>(max_accel_));

  return acceleration;
}


// #######################################################
// INSPECTION MISSIONS
// #######################################################


ackermann_msgs::msg::AckermannDriveStamped Control::static_inspection_a() {
  // Mission execution has started, calculate elapsed time from execution start
  auto current_time = this->now();
  
  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive.header.stamp = current_time;
  drive.drive.speed = 0.0;
  drive.drive.acceleration = 0.0;
  drive.drive.steering_angle = 0.0;
  drive.drive.steering_angle_velocity = 0.0;

  switch (inspection_a_phase_) {
    case A_STEERING_SWEEP: {
      double elapsed_time_1 = (current_time - driving_flag_start_time_).seconds();

      auto [target_angle_deg, sweep_done] = compute_steering_sweep(
          elapsed_time_1, inspection_steering_max_angle_, inspection_steering_sweep_duration_);

      drive.drive.steering_angle = target_angle_deg * M_PI / 180.0;

      RCLCPP_INFO(this->get_logger(), "[%d][%.2f] Steering sweep: target: %.1f deg, current: %.1f deg", inspection_a_phase_, this->now().seconds(), target_angle_deg, current_steering_angle_rad_ * 180.0 / M_PI);
      
      if (sweep_done) {
        inspection_a_phase_ = A_RPM_RAMP_UP;
        inspection_a_phase_start_time_tracker_ = current_time;
        RCLCPP_INFO(this->get_logger(), "Inspection A: Steering sweep complete. Starting RPM ramp up.");
      }
      break;
    }

    case A_RPM_RAMP_UP: {
      const double ramp_up_time = 10.0;
      double elapsed_time_2 = (current_time - inspection_a_phase_start_time_tracker_).seconds();

      if (elapsed_time_2 <= ramp_up_time) {
        inspection_target_rpm_ = (elapsed_time_2 / ramp_up_time) * inspection_a_max_rpm_;
        RCLCPP_INFO(this->get_logger(), "[%d][%.2f] RPM ramp up: target: %.1f rpm, current: %.1f rpm", inspection_a_phase_, this->now().seconds(), inspection_target_rpm_, current_rpm_);
      } else {
        inspection_a_phase_ = A_RPM_HOLD_MAX;
        RCLCPP_INFO(this->get_logger(), "Inspection A: Ramp up time finished. Holding at max RPM.");
      }
      break;
    }

    case A_RPM_HOLD_MAX: {
      const double rpm_tolerance = 5.0;
      inspection_target_rpm_ = inspection_a_max_rpm_;

      if (std::abs(current_rpm_ - inspection_a_max_rpm_) <= rpm_tolerance) {
        inspection_a_phase_ = A_RPM_RAMP_DOWN;
        inspection_a_phase_start_time_tracker_ = current_time;
        RCLCPP_INFO(this->get_logger(), "Inspection A: Reached max RPM. Starting ramp down.");
      } else {
        RCLCPP_INFO(this->get_logger(), "[%d][%.2f] Waiting to reach max RPM: target: %.1f rpm, current: %.1f rpm", inspection_a_phase_, this->now().seconds(), inspection_target_rpm_, current_rpm_);
      }
      break;
    }

    case A_RPM_RAMP_DOWN: {
      // Immediately begin decelerate and brake phase
      double brake_accel = -2.0;
      drive = decelerate_and_complete("Static Inspection A", brake_accel);
      double elapsed_decel_time = (current_time - inspection_a_phase_start_time_tracker_).seconds();
      if (mission_completion_sent_) {
        inspection_a_phase_ = A_COMPLETED;
        RCLCPP_INFO(this->get_logger(), "Inspection A: Decelerate and brake phase completed in %.2f seconds.", elapsed_decel_time);
      } else {
        RCLCPP_INFO(this->get_logger(), "[%d][%.2f] Decelerating and braking: current: %.2f rpm, %.2f m/s, elapsed: %.2f s", inspection_a_phase_, this->now().seconds(), current_rpm_, current_velocity_, elapsed_decel_time);
      }
      break;
    }

    case A_COMPLETED: {
      RCLCPP_INFO(this->get_logger(), "Inspection A: Mission completed.");
      break;
    }
  }

  // Common logic for RPM control
  if (inspection_a_phase_ != A_STEERING_SWEEP &&
     inspection_a_phase_ != A_COMPLETED &&
     inspection_a_phase_ != A_RPM_RAMP_DOWN) {
    drive.drive.acceleration = pid_rpm_control(inspection_target_rpm_, inspection_rpm_kp, inspection_rpm_ki, inspection_rpm_kd);
  }
  
  return drive;
}

ackermann_msgs::msg::AckermannDriveStamped Control::static_inspection_b() {
  // Mission execution has started, calculate elapsed time from execution start
  auto current_time = this->now();
  
  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive.header.stamp = current_time;
  drive.drive.speed = 0.0;
  drive.drive.acceleration = 0.0;
  drive.drive.steering_angle = 0.0;
  drive.drive.steering_angle_velocity = 0.0;

  switch (inspection_b_phase_) {
    case B_RPM_RAMP_UP: {
      const double ramp_up_time = 5.0;
      double rpm_elapsed = (current_time - driving_flag_start_time_).seconds();

      if (rpm_elapsed <= ramp_up_time) {
        inspection_b_target_rpm_ = (rpm_elapsed / ramp_up_time) * inspection_b_max_rpm_;
        RCLCPP_INFO(this->get_logger(), "[%d][%.2f] RPM ramp up to 50: target: %.1f rpm, current: %.1f rpm (%.1f%%)", inspection_b_phase_, this->now().seconds(), inspection_b_target_rpm_, current_rpm_, (rpm_elapsed / ramp_up_time) * 100.0);
      } else {
        inspection_b_phase_ = B_RPM_HOLD;
        inspection_b_phase_start_time_tracker_ = current_time;
        RCLCPP_INFO(this->get_logger(), "Inspection B: Ramp up time finished. Holding at 50 RPM.");
      }
      break;
    }

    case B_RPM_HOLD: {
      inspection_b_target_rpm_ = inspection_b_max_rpm_;
      const double rpm_tolerance = 2.0; // Tolerance for considering RPM at target
      
      RCLCPP_INFO(this->get_logger(), "[%d][%.2f] Waiting to reach 50 rpm: target: %.1f rpm, current: %.1f rpm", inspection_b_phase_, this->now().seconds(), inspection_b_target_rpm_, current_rpm_);
      
      // Trigger EBS once RPM reaches target
      if (std::abs(current_rpm_ - inspection_b_max_rpm_) <= rpm_tolerance && !inspection_b_ebs_triggered_) {
        inspection_b_phase_ = B_EBS_TRIGGER;
        RCLCPP_INFO(this->get_logger(), "Inspection B: Reached 50 RPM. Triggering EBS.");
      }
      break;
    }

    case B_EBS_TRIGGER: {
      if (!inspection_b_ebs_triggered_) {
        inspection_b_ebs_triggered_ = true;
        RCLCPP_INFO(this->get_logger(), "[%d][%.2f] EBS triggered, setting VCU state to AS_EMERGENCY", inspection_b_phase_, this->now().seconds());
        
        // Call EBS service
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = ebs_client->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "[%d][%.2f] EBS service called", inspection_b_phase_, this->now().seconds());
        
        sendMissionCompletion("Static Inspection B");
        RCLCPP_INFO(this->get_logger(), "Inspection B: EBS triggered. Mission completed.");
      }
      break;
    }
  }

  // Common logic for RPM control
  if (inspection_b_phase_ != B_EBS_TRIGGER) {
    drive.drive.acceleration = pid_rpm_control(inspection_b_target_rpm_, inspection_rpm_kp, inspection_rpm_ki, inspection_rpm_kd);
  }
  
  return drive;
}


ackermann_msgs::msg::AckermannDriveStamped Control::autonomous_demo_mission() {

  // Update total distance travelled
  double distance_in_phase = total_distance_travelled_ - demo_mission_phase_start_distance_;

  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive.header.stamp = this->now();
  drive.drive.steering_angle = 0.0;
  drive.drive.steering_angle_velocity = 0.0;
  drive.drive.speed = 0.0;
  drive.drive.acceleration = 0.0;

  // Mission execution has started, calculate elapsed time from execution start
  auto current_time = this->now();
  double elapsed_time = (current_time - driving_flag_start_time_).seconds();

  switch (demo_mission_phase_) {
    case DEMO_STEERING_SWEEP: {

      auto [target_angle_deg, sweep_done] = compute_steering_sweep(
          elapsed_time, inspection_steering_max_angle_, inspection_steering_sweep_duration_);

      drive.drive.steering_angle = target_angle_deg * M_PI / 180.0;

      RCLCPP_INFO(this->get_logger(), "[%d][%.2f] Steering sweep: target: %.1f deg, current: %.1f deg", demo_mission_phase_, this->now().seconds(), target_angle_deg, current_steering_angle_rad_ * 180.0 / M_PI);

      if (sweep_done) {
        RCLCPP_INFO(this->get_logger(), "Autonomous Demo: Steering sweep complete. Starting first acceleration.");
        demo_mission_phase_ = DEMO_FIRST_ACCEL;
        demo_mission_phase_start_distance_ = total_distance_travelled_;

        // Set drive to zero
        drive.drive.steering_angle = 0.0;
        drive.drive.steering_angle_velocity = 0.0;
        drive.drive.speed = 0.0;
        drive.drive.acceleration = 0.0;
      }
      break;
    }

    case DEMO_FIRST_ACCEL: {
      // Proportional acceleration control
      drive.drive.acceleration = pid_accel_control(auto_demo_target_velocity_, pid_accel_kp, pid_accel_ki, pid_accel_kd);
      drive.drive.speed = 0.0;
      drive.drive.steering_angle = 0.0;
      drive.drive.steering_angle_velocity = 0.0;

      // drive = midpoint_planning_control(auto_demo_target_velocity_, 3.0);

      RCLCPP_INFO(this->get_logger(), "[%d][%.2f] First acceleration: target: %.2f kph, current: %.2f kph", demo_mission_phase_, this->now().seconds(), auto_demo_target_velocity_ * 3.6, current_velocity_ * 3.6);

      if (distance_in_phase >= 10.0) {
        RCLCPP_INFO(this->get_logger(), "Autonomous Demo: First acceleration complete (10m). Speed: %.2f kph", current_velocity_ * 3.6);
        demo_mission_phase_ = DEMO_STOPPING;
        demo_mission_phase_start_distance_ = total_distance_travelled_;
      }
      break;
    }

    case DEMO_STOPPING: {
      drive.drive.acceleration = -5.0; // Brake
      drive.drive.speed = 0.0;
      drive.drive.steering_angle = 0.0;
      drive.drive.steering_angle_velocity = 0.0;

      RCLCPP_INFO(this->get_logger(), "[%d][%.2f] Stopping: target: %.2f kph, current: %.2f kph", demo_mission_phase_, this->now().seconds(), 0.0, current_velocity_ * 3.6);

      if (velx <= 0.1) {
        if (velx > 0.1 && distance_in_phase >= 10.0) {
          RCLCPP_WARN(this->get_logger(), "Autonomous Demo: Failed to stop within 10m. Traveled %.2f m.", distance_in_phase);
        } else {
          RCLCPP_INFO(this->get_logger(), "Autonomous Demo: Stop complete at %.2f meters.", distance_in_phase);
        }
        demo_mission_phase_ = DEMO_SECOND_ACCEL;
        demo_mission_phase_start_distance_ = total_distance_travelled_;
      }
      break;
    }

    case DEMO_SECOND_ACCEL: {
      // Proportional acceleration control
      drive.drive.acceleration = pid_accel_control(auto_demo_target_velocity_, pid_accel_kp, pid_accel_ki, pid_accel_kd);
      drive.drive.speed = 0.0;
      drive.drive.steering_angle = 0.0;
      drive.drive.steering_angle_velocity = 0.0;

      // drive = midpoint_planning_control(auto_demo_target_velocity_, 3.0);

      RCLCPP_INFO(this->get_logger(), "[%d][%.2f] Second acceleration: target: %.2f kph, current: %.2f kph", demo_mission_phase_, this->now().seconds(), auto_demo_target_velocity_ * 3.6, current_velocity_ * 3.6);

      if (distance_in_phase >= 10.0) {
        RCLCPP_INFO(this->get_logger(), "Autonomous Demo: Second acceleration complete (10m). Speed: %.2f kph", velx * 3.6);
        demo_mission_phase_ = DEMO_EBS_AND_COMPLETE;
      }
      break;
    }

    case DEMO_EBS_AND_COMPLETE: {
      if (!demo_mission_ebs_triggered_) {
        demo_mission_ebs_triggered_ = true;
        RCLCPP_INFO(this->get_logger(), "Autonomous Demo: Deploying EBS.");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = ebs_client->async_send_request(request);
      }
      RCLCPP_INFO(this->get_logger(), "Autonomous Demo: EBS triggered. Mission completed.");
      break;
    }
  }
  return drive;
}


// Helper for steering sweep math (returns {target_angle_deg, sweep_done})
std::pair<double, bool> Control::compute_steering_sweep(
    double elapsed_time,
    double max_angle_deg,
    double sweep_duration) const {
  // Use class constants for durations if available
  double approach = 3.0;
  double departure = 3.0;
  double sweep_main = sweep_duration - (approach + departure);
  double target_angle_deg = 0.0;
  bool sweep_done = false;

  if (elapsed_time < approach) {
    double eased_progress = 0.5 * (1.0 - cos((elapsed_time / approach) * M_PI));
    target_angle_deg = max_angle_deg * eased_progress;
  } else if (elapsed_time < approach + sweep_main) {
    double eased_progress = 0.5 * (1.0 - cos(((elapsed_time - approach) / sweep_main) * M_PI));
    target_angle_deg = max_angle_deg * (1.0 - 2.0 * eased_progress);
  } else if (elapsed_time < approach + sweep_main + departure) {
    double eased_progress = 0.5 * (1.0 - cos(((elapsed_time - approach - sweep_main) / departure) * M_PI));
    target_angle_deg = -max_angle_deg * (1.0 - eased_progress);
  } else {
    sweep_done = true;
    target_angle_deg = 0.0;
  }
  return {target_angle_deg, sweep_done};
}


// #######################################################
// SYSTEM CHARACTERIZATION
// #######################################################

ackermann_msgs::msg::AckermannDriveStamped Control::system_characterization_mission() {

  double step_target_steering_angle_deg = 15.0; // 
  double step_target_rpm = 200.0; // approx. 5 m/s
  double step_duration = 3.0; // seconds
  double hold_time = 3.0;
  int char_num_repetitions_ = 5; // Number of repetitions for each step


  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive = step_input_characterization(step_target_steering_angle_deg, step_target_rpm, step_duration, hold_time, char_num_repetitions_);

  return drive;
}

ackermann_msgs::msg::AckermannDriveStamped Control::step_input_characterization(double target_steering_angle_deg, double target_rpm, double step_duration, double hold_time, int char_num_repetitions_) {
  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive.header.stamp = this->now();
  
  // Initialize if this is the first call
  if (char_phase_ == CHAR_STEERING_STEP && !char_phase_started_) {
    char_phase_start_time_ = this->now();
    step_phase_start_time_ = this->now();
    char_current_repetition_ = 0;
    RCLCPP_INFO(this->get_logger(), "Starting steering step input characterization with %d repetitions", char_num_repetitions_);
  }
  char_phase_started_ = true;
  
  auto current_time = this->now();
  double phase_elapsed_time = (current_time - char_phase_start_time_).seconds();
  double step_elapsed_time = (current_time - step_phase_start_time_).seconds();
  // Convert target steering angle to radians
  char_target_steering_ = target_steering_angle_deg * M_PI / 180.0;
  // Initialize variables
  double target_angle = 0.0;
  
  switch (char_phase_) {
    case CHAR_STEERING_STEP: {
      // Handle steering step repetitions
      switch (step_phase_) {
        case STEP_WAIT_ZERO: {
          // Wait at zero for step_duration
          drive.drive.steering_angle = 0.0;
          drive.drive.steering_angle_velocity = 0.0;
          drive.drive.speed = 0.0;
          drive.drive.acceleration = 0.0;
          
          if (step_elapsed_time >= step_duration) {
            step_phase_ = STEP_APPLY_TARGET;
            step_phase_start_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "Steering repetition %d/%d: Applying target", char_current_repetition_ + 1, char_num_repetitions_);
          }
          break;
        }
        
        case STEP_APPLY_TARGET: {
          // Apply target steering angle using pure pursuit model
          // Calculate pure pursuit steering angle for target point
          double target_distance = lookahead_distance_;
          target_angle = atan2(char_target_steering_, target_distance);
          
          // Calculate error as difference between pure pursuit steering and current steering
          drive.drive.steering_angle = target_angle;
          drive.drive.steering_angle_velocity = 0.0;
          drive.drive.speed = 0.0;
          drive.drive.acceleration = 0.0;
          
          if (step_elapsed_time >= step_duration) {
            step_phase_ = STEP_HOLD_TARGET;
            step_phase_start_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "Steering repetition %d/%d: Holding target for 10s", char_current_repetition_ + 1, char_num_repetitions_);
          }
          break;
        }
        
        case STEP_HOLD_TARGET: {
          // Hold target using pure pursuit model
          // Calculate pure pursuit steering angle for target point
          double target_distance = lookahead_distance_;
          target_angle = atan2(char_target_steering_, target_distance);
          
          // Calculate error as difference between pure pursuit steering and current steering
          drive.drive.steering_angle = target_angle;
          drive.drive.steering_angle_velocity = 0.0;
          drive.drive.speed = 0.0;
          drive.drive.acceleration = 0.0;
          
          if (step_elapsed_time >= hold_time) {
            step_phase_ = STEP_RETURN_ZERO;
            step_phase_start_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "Steering repetition %d/%d: Returning to zero", char_current_repetition_ + 1, char_num_repetitions_);
          }
          break;
        }
        
        case STEP_RETURN_ZERO: {
          // Return to zero
          drive.drive.steering_angle = 0.0;
          drive.drive.steering_angle_velocity = 0.0;
          drive.drive.speed = 0.0;
          drive.drive.acceleration = 0.0;
          
          if (step_elapsed_time >= step_duration) {
            char_current_repetition_++;
            if (char_current_repetition_ >= char_num_repetitions_) {
              // All steering repetitions complete, move to RPM phase
              char_phase_ = CHAR_RPM_STEP;
              char_phase_start_time_ = current_time;
              step_phase_start_time_ = current_time;
              step_phase_ = STEP_WAIT_ZERO;
              char_current_repetition_ = 0;
              char_target_rpm_ = target_rpm;
              char_steering_step_complete_ = true;
              RCLCPP_INFO(this->get_logger(), "Steering phase complete. Starting RPM step characterization with %d repetitions", char_num_repetitions_);
            } else {
              // Start next repetition
              step_phase_ = STEP_WAIT_ZERO;
              step_phase_start_time_ = current_time;
              RCLCPP_INFO(this->get_logger(), "Steering repetition %d/%d complete. Starting next repetition", char_current_repetition_, char_num_repetitions_);
            }
          }
          break;
        }
      }
      
      // Log steering data for all phases
      std::vector<std::string> headers = {"time", "repetition", "step_phase", "target_steering_deg", "current_steering_deg"};
      std::vector<double> values = {
        phase_elapsed_time,
        static_cast<double>(char_current_repetition_),
        static_cast<double>(step_phase_),
        target_angle * 180.0 / M_PI,
        current_steering_angle_rad_ * 180.0 / M_PI,
      };
      logDatatoCSV("steering_step_characterization", headers, values);
      break;
    }
    
    case CHAR_RPM_STEP: {
      // Handle RPM step repetitions
      switch (step_phase_) {
        case STEP_WAIT_ZERO: {
          // Wait at zero for step_duration
          drive.drive.steering_angle = 0.0;
          drive.drive.steering_angle_velocity = 0.0;
          drive.drive.speed = 0.0;
          drive.drive.acceleration = 0.0;
          
          if (step_elapsed_time >= step_duration) {
            step_phase_ = STEP_APPLY_TARGET;
            step_phase_start_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "RPM repetition %d/%d: Applying target", char_current_repetition_ + 1, char_num_repetitions_);
          }
          break;
        }
        
        case STEP_APPLY_TARGET: {
          // Apply target RPM using PID control
          drive.drive.steering_angle = 0.0;
          drive.drive.steering_angle_velocity = 0.0;
          drive.drive.speed = 0.0;
          drive.drive.acceleration = pid_rpm_control(char_target_rpm_, inspection_rpm_kp, inspection_rpm_ki, inspection_rpm_kd);
          
          if (step_elapsed_time >= step_duration) {
            step_phase_ = STEP_HOLD_TARGET;
            step_phase_start_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "RPM repetition %d/%d: Holding target for 10s", char_current_repetition_ + 1, char_num_repetitions_);
          }
          break;
        }
        
        case STEP_HOLD_TARGET: {
          // Hold target RPM
          drive.drive.steering_angle = 0.0;
          drive.drive.steering_angle_velocity = 0.0;
          drive.drive.speed = 0.0;
          drive.drive.acceleration = pid_rpm_control(char_target_rpm_, inspection_rpm_kp, inspection_rpm_ki, inspection_rpm_kd);
          
          if (step_elapsed_time >= hold_time) {
            step_phase_ = STEP_RETURN_ZERO;
            step_phase_start_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "RPM repetition %d/%d: Returning to zero", char_current_repetition_ + 1, char_num_repetitions_);
          }
          break;
        }
        
        case STEP_RETURN_ZERO: {
          // Return to zero RPM
          drive.drive.steering_angle = 0.0;
          drive.drive.steering_angle_velocity = 0.0;
          drive.drive.speed = 0.0;
          drive.drive.acceleration = 0.0;
          
          if (step_elapsed_time >= step_duration) {
            char_current_repetition_++;
            if (char_current_repetition_ >= char_num_repetitions_) {
              // All RPM repetitions complete, finish characterization
              char_phase_ = CHAR_COMPLETED;
              char_rpm_step_complete_ = true;
            } else {
              // Start next repetition
              step_phase_ = STEP_WAIT_ZERO;
              step_phase_start_time_ = current_time;
              RCLCPP_INFO(this->get_logger(), "RPM repetition %d/%d complete. Starting next repetition", char_current_repetition_, char_num_repetitions_);
            }
          }
          break;
        }
      }
      
      // Log RPM data for all phases
      std::vector<std::string> headers = {"time", "repetition", "step_phase", "target_rpm", "current_rpm", "acceleration"};
      std::vector<double> values = {
        phase_elapsed_time,
        static_cast<double>(char_current_repetition_),
        static_cast<double>(step_phase_),
        char_target_rpm_,
        current_rpm_,
        acceleration,
      };
      logDatatoCSV("rpm_step_characterization", headers, values);
      break;
    }
    
    case CHAR_COMPLETED: {
      // Keep vehicle stationary
      drive.drive.steering_angle = 0.0;
      drive.drive.steering_angle_velocity = 0.0;
      drive.drive.speed = 0.0;
      drive.drive.acceleration = 0.0;

      sendMissionCompletion("System Characterization");
      RCLCPP_INFO(this->get_logger(), "System characterization completed. Vehicle stationary.");
      break;
    }
  }
  
  return drive;
}



// #######################################################
// VISUALISATION
// #######################################################


void Control::publishPFConeVisualization() {
    if (!bristol_fsai_debug_ || !cone_pub) {
        return;
    }
    // Add cone markers
    conePubList = visualization_msgs::msg::MarkerArray();
    int id = 0;
    for (const auto& cone : coneList) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_footprint";
        marker.header.stamp = this->now();
        marker.ns = "cones";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Cone position (2D, z = 0)
        marker.pose.position.x = cone.x;
        marker.pose.position.y = cone.y;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;

        // Cone size
        marker.scale.x = 0.3;  // Diameter
        marker.scale.y = 0.3;
        marker.scale.z = 0.5;  // Height

        // Cone color (RGBA) - color based on cone type
        if (cone.getColour() == BLUE) {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        } else if (cone.getColour() == YELLOW) {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (cone.getColour() == ORANGE){
            // Default red color for unknown cone types
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else{
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        }
        marker.color.a = 0.6;  // More transparent

        // Lifetime is proportional to how "alive" the cone is.
        // A cone is removed when deadTimer >= max_dead_time_.
        // The lifetime is based on the measured time between callbacks (last_cb_dt_).
        double lifetime_seconds = std::max(0.0, max_dead_time_ - cone.deadTimer);
        marker.lifetime = rclcpp::Duration::from_seconds(lifetime_seconds);
        conePubList.markers.push_back(marker);
    }
    // Only publish if we have markers to publish
    if (!conePubList.markers.empty()) {
        cone_pub->publish(conePubList);
    }
}

void Control::publishMidpointVisualization() {
    if (!bristol_fsai_debug_ || !midpoint_pub_) {
        return;
    }
    // If the midpoint is not valid, publish a DELETE marker to remove it.
    if (!valid_midpoint) {
        if (midpoint_marker_initialized_) {
            midpoint_marker_.action = visualization_msgs::msg::Marker::DELETE;
            midpoint_pub_->publish(midpoint_marker_);
        }
        return;
    }

    // Add midpoint marker
    if (!midpoint_marker_initialized_) {
        midpoint_marker_.header.frame_id = "base_footprint";
        midpoint_marker_.ns = "midpoints";
        midpoint_marker_.id = 0; // Fixed ID for the single marker on this topic
        midpoint_marker_.type = visualization_msgs::msg::Marker::SPHERE;
        midpoint_marker_.scale.x = 0.5;  // Diameter
        midpoint_marker_.scale.y = 0.5;
        midpoint_marker_.scale.z = 0.5;
        midpoint_marker_.color.r = 0.0;
        midpoint_marker_.color.g = 1.0;
        midpoint_marker_.color.b = 0.0;
        midpoint_marker_.color.a = 0.8;  // Slightly transparent
        midpoint_marker_initialized_ = true;
    }

    midpoint_marker_.action = visualization_msgs::msg::Marker::ADD;
    midpoint_marker_.header.stamp = this->now();
    midpoint_marker_.pose.position.x = mid_x;
    midpoint_marker_.pose.position.y = mid_y;
    midpoint_marker_.pose.position.z = 0.5;  // Slightly higher than cones
    midpoint_marker_.pose.orientation.w = 1.0;
    midpoint_marker_.lifetime = rclcpp::Duration::from_seconds(1.0);
    
    midpoint_pub_->publish(midpoint_marker_);
}

void Control::publishSplineTrajectory() {
    if (!bristol_fsai_debug_ || !spline_pub_) {
        return;
    }
    visualization_msgs::msg::MarkerArray marker_array;

    // Trajectory marker
    visualization_msgs::msg::Marker spline_marker;
    spline_marker.header.frame_id = "base_footprint";
    spline_marker.header.stamp = this->now();
    spline_marker.ns = "spline_trajectory";
    spline_marker.id = 0;
    spline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    
    if (spline_points_.empty()) {
        spline_marker.action = visualization_msgs::msg::Marker::DELETE;
    } else {
        spline_marker.action = visualization_msgs::msg::Marker::ADD;
        spline_marker.pose.orientation.w = 1.0;
        spline_marker.scale.x = 0.05; // Line width
        spline_marker.color.b = 1.0;
        spline_marker.color.a = 1.0;
        spline_marker.points = spline_points_;
        spline_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    }
    marker_array.markers.push_back(spline_marker);

    // Lookahead point marker
    visualization_msgs::msg::Marker lookahead_marker;
    lookahead_marker.header.frame_id = "base_footprint";
    lookahead_marker.header.stamp = this->now();
    lookahead_marker.ns = "spline_trajectory";
    lookahead_marker.id = 1;
    lookahead_marker.type = visualization_msgs::msg::Marker::SPHERE;
    if (spline_points_.empty()) {
        lookahead_marker.action = visualization_msgs::msg::Marker::DELETE;
    } else {
        lookahead_marker.action = visualization_msgs::msg::Marker::ADD;
        lookahead_marker.pose.position = spline_lookahead_point_;
        lookahead_marker.pose.orientation.w = 1.0;
        lookahead_marker.scale.x = 0.2;
        lookahead_marker.scale.y = 0.2;
        lookahead_marker.scale.z = 0.2;
        lookahead_marker.color.r = 1.0;
        lookahead_marker.color.g = 0.5;
        lookahead_marker.color.b = 0.0;
        lookahead_marker.color.a = 1.0;
        lookahead_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    }
    marker_array.markers.push_back(lookahead_marker);

    // Midpoint markers
    for (size_t i = 0; i < spline_midpoints_.size(); ++i) {
        visualization_msgs::msg::Marker midpoint_marker;
        midpoint_marker.header.frame_id = "base_footprint";
        midpoint_marker.header.stamp = this->now();
        midpoint_marker.ns = "spline_midpoints";
        midpoint_marker.id = i;
        midpoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
        midpoint_marker.action = visualization_msgs::msg::Marker::ADD;
        midpoint_marker.pose.position.x = spline_midpoints_[i].first;
        midpoint_marker.pose.position.y = spline_midpoints_[i].second;
        midpoint_marker.pose.position.z = 0;
        midpoint_marker.pose.orientation.w = 1.0;
        midpoint_marker.scale.x = 0.15; // Smaller than lookahead point
        midpoint_marker.scale.y = 0.15;
        midpoint_marker.scale.z = 0.15;
        midpoint_marker.color.r = 0.0;
        midpoint_marker.color.g = 1.0;
        midpoint_marker.color.b = 0.0;
        midpoint_marker.color.a = 0.7; // Semi-transparent
        midpoint_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_array.markers.push_back(midpoint_marker);
    }

    spline_pub_->publish(marker_array);
}

void Control::publishStopZoneVisualization() {
    if (!bristol_fsai_debug_ || !stop_zone_pub_) {
        return;
    }
    if (!stop_zone_marker_initialized_) {
        stop_zone_marker_.header.frame_id = "odom_tmp";
        stop_zone_marker_.ns = "stop_zone";
        stop_zone_marker_.id = 0;
        stop_zone_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
        stop_zone_marker_.scale.x = LAP_COMPLETION_RADIUS * 2;  // Diameter
        stop_zone_marker_.scale.y = LAP_COMPLETION_RADIUS * 2;
        stop_zone_marker_.scale.z = 0.0;  // Height
        stop_zone_marker_.color.r = 1.0;
        stop_zone_marker_.color.g = 0.0;
        stop_zone_marker_.color.b = 0.0;
        stop_zone_marker_.color.a = 0.5;  // Semi-transparent
        stop_zone_marker_initialized_ = true;
    }
    stop_zone_marker_.header.stamp = this->now();
    stop_zone_marker_.pose.position.x = lap_start_x_;
    stop_zone_marker_.pose.position.y = lap_start_y_;
    stop_zone_marker_.pose.position.z = 0.05;
    stop_zone_marker_.pose.orientation.w = 1.0;
    stop_zone_marker_.lifetime = rclcpp::Duration::from_seconds(0.5);
    stop_zone_pub_->publish(stop_zone_marker_);
}

// #########################################################
// Misc
// #########################################################

void Control::logDatatoCSV(const std::string& filename, const std::vector<std::string>& headers, const std::vector<double>& values) {
    // EXAMPLE USAGE:
    // std::vector<std::string> headers = {"time", "target_rpm", "current_rpm"};
    // std::vector<double> values = {this->now().seconds(), inspection_b_target_rpm_, current_rpm_};
    // logDatatoCSV("inspection_b_rpm", headers, values);
    
    // save to a csv
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::string time_string = std::to_string(tm.tm_hour) + "-" + std::to_string(tm.tm_min);
    std::string log_filename = "data-logs/" + filename + "_" + time_string + ".csv";
    
    // Check if data-logs directory exists, create if it doesn't
    std::filesystem::path data_logs_dir = "data-logs";
    if (!std::filesystem::exists(data_logs_dir)) {
        std::filesystem::create_directory(data_logs_dir);
    }
    
    if (!csv_headers_written_) {
      log_file_.open(log_filename);
      
      // Write headers
      for (size_t i = 0; i < headers.size(); ++i) {
        log_file_ << headers[i];
        if (i < headers.size() - 1) {
          log_file_ << ",";
        }
      }
      log_file_ << "\n";
      csv_headers_written_ = true;
    }
    
    if (log_file_.is_open()) {
      // Write values
      for (size_t i = 0; i < values.size(); ++i) {
        log_file_ << values[i];
        if (i < values.size() - 1) {
          log_file_ << ",";
        }
      }
      log_file_ << "\n";
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Control>());
    rclcpp::shutdown();
    return 0;
}