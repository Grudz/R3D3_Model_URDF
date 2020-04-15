#include <igvc_flatland/AutoJudge.h>
#include <igvc_flatland/WaypointList.h>
#include <pluginlib/class_list_macros.h>
#include <boost/filesystem.hpp>

using namespace flatland_server;

namespace flatland_plugins
{

void AutoJudge::OnInitialize(const YAML::Node &config)
{
  // Load settings from YAML
  LoadConfig(config);

  // Initialize pointers to Flatland physics bodies
  base_body_ = GetModel()->GetBody("base");
  if (base_body_ == nullptr) {
    throw YAMLException("Body with name [base] does not exist");
  }
  left_wheel_body_ = GetModel()->GetBody("left_wheel");
  if (left_wheel_body_ == nullptr) {
    throw YAMLException("Body with name [left_wheel] does not exist");
  }
  right_wheel_body_ = GetModel()->GetBody("right_wheel");
  if (right_wheel_body_ == nullptr) {
    throw YAMLException("Body with name [right_wheel] does not exist");
  }

  // Initialize ROS bits
  sub_fix_ = nh_.subscribe("gps/fix", 1, &AutoJudge::recvFix, this);
  sub_cmd_ = nh_.subscribe("cmd_vel", 1, &AutoJudge::recvCmd, this);
  pub_waypoints_ = nh_.advertise<igvc_flatland::WaypointList>("waypoints", 1, true);
  pub_waypoint_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoint_markers", 1);
  marker_timer_ = nh_.createTimer(ros::Duration(0.1), &AutoJudge::markerTimerCb, this);

  // Initialize properties
  left_wheel_on_line_ = false;
  right_wheel_on_line_ = false;
  line_crossing_state_ = NO_CROSSING;
  delay_ = 0.0;

  run_active_ = true;
  received_cmd_ = false;
  run_phase_ = LANE_1;
  collision_ = false;
  run_time_ = 0.0;
  no_progress_time_ = 0.0;
  distance_penalty_ = 0.0;

  dist_ref_.Set(INFINITY, INFINITY);
  last_dist_ref_.Set(INFINITY, INFINITY);
  latched_dist_ = 0.0;
  current_dist_2_ = 0.0;
  going_backward_ = false;
  lane_1_dist_ = 235.0;

  start_point_.Set(INFINITY, INFINITY);
  start_heading_ = INFINITY;
  at_start_ = false;
}

void AutoJudge::BeforePhysicsStep(const Timekeeper &timekeeper)
{
  b2Vec2 vehicle_pos = base_body_->physics_body_->GetPosition();

  // Nothing to do if run is over, or hasn't started yet
  if (!run_active_ || !received_cmd_) {
    return;
  }

  // Latch starting heading to determine which direction is correct
  if (!std::isfinite(start_heading_)) {
    start_heading_ = base_body_->physics_body_->GetAngle();
    if (fabs(start_heading_ - M_PI_2) < 0.1) {
      first_opening_ = "north_opening";
      second_opening_ = "south_opening";
    } else {
      first_opening_ = "south_opening";
      second_opening_ = "north_opening";
    }
  }

  // Check total time and end run if time limit is exceeded
  if (run_time_ == 0.0) {
    ROS_INFO("Run starts now!");
  }
  run_time_ += timekeeper.GetStepSize();
  if (run_time_ >= time_limit_) {
    run_active_ = false;
    std::stringstream ss;
    ss.precision(4);
    ss << (int)floor(time_limit_ / 60.0) << "m" << fmod(time_limit_, 60.0) << "s";
    ROS_ERROR("Time limit of %s exceeded; end of run!", ss.str().c_str());
    DisplayScore();
  }

  // Check collision status and end run if true
  if (collision_) {
    run_active_ = false;
    ROS_WARN("10 ft. penalty incurred for colliding with obstacle");
    distance_penalty_ += 10.0;
    ROS_ERROR("Robot collided with obstacle; end of run!");
    DisplayScore();
  }

  // Update line crossing state machine and end the run if robot left the course
  IterateLineCrossingDetection(timekeeper);
  if (line_crossing_state_ == EXITED_COURSE) {
    run_active_ = false;
    ROS_WARN("10 ft. penalty incurred for leaving the course");
    distance_penalty_ += 10.0;
    ROS_ERROR("Robot left the course; end of run!");
    DisplayScore();
  }

  // Check if vehicle is within 2 meters of start
  at_start_ = (start_point_ - vehicle_pos).LengthSquared() < 2.0 * 2.0;

  // Detect phase changes
  switch (run_phase_) {
    case LANE_1:
      // Stays in this state until first opening waypoint is hit
      if (waypoints_[first_opening_].hit) {
        // Transition to NO_MANS_LAND state and reset distance
        latched_dist_ = 0.0;
        dist_ref_.Set(INFINITY, INFINITY);
        last_dist_ref_.Set(INFINITY, INFINITY);
        no_progress_time_ = 0.0;
        run_phase_ = NO_MANS_LAND;
      }
      break;
    case NO_MANS_LAND:
      // Stays in this state until robot hits second opening waypoint to
      // leave no man's land
      if (waypoints_[second_opening_].hit) {
        run_phase_ = LANE_2;
      }
      break;
    case LANE_2:
      // Stays in this state until the robot returns to the start
      if (at_start_) {
        run_phase_ = DONE;
        ROS_INFO("Course Completed!");
        DisplayScore();
      }
      break;
    case DONE:
      // Shouldn't get here, but just in case...
      DisplayScore();
      break;
  }

  /* Update run distance when in a lined section */
  if (run_phase_ != NO_MANS_LAND) {
    if (!dist_ref_.IsValid()) {
      dist_ref_ = vehicle_pos;
    }
    current_dist_2_ = (vehicle_pos - dist_ref_).LengthSquared();
    no_progress_time_ += timekeeper.GetStepSize();

    // Move the chains when vehicle moves more than 5 meters from latched point
    if (current_dist_2_ > (5.0 * 5.0)) {
      double last_dist_2;
      if (last_dist_ref_.IsValid()) {
        last_dist_2 = (vehicle_pos - last_dist_ref_).LengthSquared();
      } else {
        last_dist_2 = 0.0;
      }

      // If vehicle is closer to past latched point here, that means it is going backward
      if (!last_dist_ref_.IsValid() || (last_dist_2 > current_dist_2_)) {
        // Moving forward; update latched points, latch current distance score, reset no progress time
        last_dist_ref_ = dist_ref_;
        dist_ref_ = vehicle_pos;
        latched_dist_ += 3.28 * sqrt(current_dist_2_);
        no_progress_time_ = 0.0;
        going_backward_ = false;
      } else {
        // Moving backward; ignore distance score updates and raise the flag
        going_backward_ = true;
      }
    }
  }

  // End the run if no progress has been made in 1 minute
  if (no_progress_time_ > 60.0) {
    run_active_ = false;
    ROS_ERROR("No progress made in 1 minute; end of run!");
    DisplayScore();
  }

  // Update waypoint hits
  CheckWheelWaypointHit(left_wheel_body_);
  CheckWheelWaypointHit(right_wheel_body_);
}

void AutoJudge::IterateLineCrossingDetection(const Timekeeper &timekeeper)
{
  // Transition flags from not being on a line
  bool left_rising_edge = CheckLeftLineCrossing();
  bool right_rising_edge = CheckRightLineCrossing();

  // State machine to detect line crossings and lane departures
  switch (line_crossing_state_) {
    case NO_CROSSING: // Nominal state
      delay_ = 0.0;
      if (left_rising_edge && right_rising_edge) {
        line_crossing_state_ = EXITED_COURSE;
      } else if (left_rising_edge) {
        line_crossing_state_ = LEFT_CROSSED;
      } else if (right_rising_edge) {
        line_crossing_state_ = RIGHT_CROSSED;
      }
      break;
    case LEFT_CROSSED: // Left wheel hit a line
      delay_ += timekeeper.GetStepSize();
      if (right_rising_edge) {
        // Right wheel also hit a line; lane departure detected
        line_crossing_state_ = EXITED_COURSE;
      } else if (left_rising_edge) {
        // Left wheel hit a line again; Assume robot re-entered course
        line_crossing_state_ = NO_CROSSING;
        ROS_WARN("5 ft. penalty incurred for careless driving");
        distance_penalty_ += 5.0;
      } else if (delay_ >= 10.0) {
        // 10 seconds since a line contact; assume robot re-entered course
        line_crossing_state_ = NO_CROSSING;
        ROS_WARN("5 ft. penalty incurred for careless driving");
        distance_penalty_ += 5.0;
      }
      break;
    case RIGHT_CROSSED: // Right wheel hit a line
      delay_ += timekeeper.GetStepSize();
      if (left_rising_edge) {
        // Left wheel also hit a line; lane departure detected
        line_crossing_state_ = EXITED_COURSE;
      } else if (right_rising_edge) {
        // Right wheel hit a line again; Assume robot re-entered course
        line_crossing_state_ = NO_CROSSING;
        ROS_WARN("5 ft. penalty incurred for careless driving");
        distance_penalty_ += 5.0;
      } else if (delay_ >= 10.0) {
        // 10 seconds since a line contact; assume robot re-entered course
        line_crossing_state_ = NO_CROSSING;
        ROS_WARN("5 ft. penalty incurred for careless driving");
        distance_penalty_ += 5.0;
      }
      break;
    case EXITED_COURSE: // Robot left the course; run over
      break;
  }
}

bool AutoJudge::CheckLeftLineCrossing()
{
  bool rising_edge;
  bool line_status = GetLineStatus(left_wheel_body_);
  if (!left_wheel_on_line_ && line_status) {
    rising_edge = true;
  } else {
    rising_edge = false;
  }
  left_wheel_on_line_ = line_status;
  return rising_edge;
}

bool AutoJudge::CheckRightLineCrossing()
{
  bool rising_edge;
  bool line_status = GetLineStatus(right_wheel_body_);
  if (!right_wheel_on_line_ && line_status) {
    rising_edge = true;
  } else {
    rising_edge = false;
  }
  right_wheel_on_line_ = line_status;
  return rising_edge;
}

bool AutoJudge::GetLineStatus(Body *body)
{
  b2Vec2 pos = body->physics_body_->GetPosition();
  int mx = std::max(0, std::min((int)((pos.x - x0_) / resolution_), lines_image_.cols - 1));
  int my = std::max(0, std::min((int)((y0_ - pos.y) / resolution_), lines_image_.rows - 1));
  return lines_image_.at<float>(my, mx) < 0.5;
}

void AutoJudge::CheckWheelWaypointHit(Body *body)
{
  for (WaypointDict::iterator it = waypoints_.begin(); it != waypoints_.end(); ++it) {
    double d2 = (it->second.map_coords - body->GetPhysicsBody()->GetPosition()).LengthSquared();
    if (d2 < (it->second.dist_thres * it->second.dist_thres) && !it->second.hit) {
      if (it->first == second_opening_ && run_phase_ == LANE_1) {
        // Robot went backward and entered no man's land on the wrong side
        ROS_WARN_THROTTLE(5.0, "Robot went the wrong way!");
      } else {
        if (it->first == first_opening_) {
          // Special message for the first waypoint entering no man's land
          ROS_INFO("Entering no man's land");
        } else if (it->first == second_opening_) {
          // Special message for the last waypoint leaving no man's land
          ROS_INFO("Leaving no man's land");
        } else {
          ROS_INFO("Hit waypoint [%s]", it->first.c_str());
        }
        it->second.hit = true;
      }
    }
  }
}

void AutoJudge::DisplayScore()
{
  double adjusted_run_distance;
  double adjusted_run_time;
  double section_dist;

  // Final score is dependent on the state of the run when the end condition is triggered
  std::stringstream ss;
  ss.precision(4);
  switch (run_phase_) {
    case LANE_1: // First lined section before no man's land
      // Score is the amount of distance traveled, ignoring any backward progress
      adjusted_run_distance = going_backward_ ? latched_dist_ : latched_dist_ + 3.28 * sqrt(current_dist_2_);

      // Offset any accumulated distance penalties
      adjusted_run_distance -= distance_penalty_;

      // Output score information
      ss << "Run score: " << adjusted_run_distance << " ft.";
      if (distance_penalty_ > 0.0) {
        ss << " with " << distance_penalty_ << " ft. of penalties";
      }
      break;
    case NO_MANS_LAND: // Reached the first opening and started exploring no man's land
      // Carry-over distance from the first lined section
      adjusted_run_distance = lane_1_dist_;

      // Add a flat 100 ft. per waypoint that was hit
      adjusted_run_distance += (CountHitWaypoints() - 1) * 100.0;

      // Offset any accumulated distance penalties
      adjusted_run_distance -= distance_penalty_;

      // Output score information
      ss << "Run score: " << adjusted_run_distance << " ft.";
      if (distance_penalty_ > 0.0) {
        ss << " with " << distance_penalty_ << " ft. of penalties\n";
      }
      ss << "Completed first lined section (" << lane_1_dist_ << " ft.)";
      if (CountHitWaypoints() == 2) {
        ss << "\nHit " << CountHitWaypoints() - 1 << " waypoint (" << (CountHitWaypoints() - 1) * 100.0 << " ft.)";
      } else if (CountHitWaypoints() > 2) {
        ss << "\nHit " << CountHitWaypoints() - 1 << " waypoints (" << (CountHitWaypoints() - 1) * 100.0 << " ft.)";
      }
      break;
    case LANE_2: // Re-entered the course and started the second lined section
      // Carry-over distance from the first lined section and a flat 100 ft. score for each completed waypoint
      adjusted_run_distance = lane_1_dist_ + (CountHitWaypoints() - 1) * 100.0;

      // Compute and add the distance completed since entering the second lined section,
      // ignoring any backward progress
      section_dist = (going_backward_ ? latched_dist_ : latched_dist_ + 3.28 * sqrt(current_dist_2_));
      adjusted_run_distance += section_dist;

      // Offset any accumulated distance penalties
      adjusted_run_distance -= distance_penalty_;

      // Output score information
      ss << "Run score: " << adjusted_run_distance << " ft.";
      if (distance_penalty_ > 0.0) {
        ss << " with " << distance_penalty_ << " ft. of penalties";
      }
      ss << "\nCompleted first lined section (" << lane_1_dist_ << " ft.)";
      if (AllWaypointsHit()) {
        ss << "\nHit all waypoints (" << (waypoints_.size() - 1) * 100.0 << " ft.)";
      } else if (CountHitWaypoints() == 2) {
        ss << "\nHit " << CountHitWaypoints() - 1 << " waypoint (" << (CountHitWaypoints() - 1) * 100.0 << " ft.)";
      } else if (CountHitWaypoints() > 2) {
        ss << "\nHit " << CountHitWaypoints() - 1 << " waypoints (" << (CountHitWaypoints() - 1) * 100.0 << " ft.)";
      }
      ss << "\nDistance after entering second lined section: " << section_dist << " ft.";
      break;
    case DONE:
      // If course is completed, score is final time increased by distance penalty value (1 ft. = 1 sec)
      adjusted_run_time = run_time_ + distance_penalty_;

      // Output score information
      ss << "Run score: Completed in " << (int)floor(adjusted_run_time / 60.0) << "m" << fmod(adjusted_run_time, 60.0) << "s";
      if (distance_penalty_ > 0.0) {
        ss << " with " << distance_penalty_ << " seconds of penalties";
        if (CountMissedWaypoints()) {
          ss << " and " << CountMissedWaypoints() << " missed waypoints";
        }
      } else if (CountMissedWaypoints()) {
        ss << " with " << CountMissedWaypoints() << " missed waypoints";
      }

      // Run over!
      run_active_ = false;
      break;
  }

  // Actually display score in terminal
  ROS_INFO("%s", ss.str().c_str());
}

void AutoJudge::BeginContact(b2Contact *contact)
{
  // For some reason, the code hits here a few times immediately on startup.
  // Prevent the run from ending by waiting 2 seconds before registering
  // collisions.
  if (run_time_ > 2.0) {
    collision_ = true;
  }
}

void AutoJudge::recvCmd(const geometry_msgs::TwistConstPtr &msg)
{
  received_cmd_ = true;
}

void AutoJudge::recvFix(const sensor_msgs::NavSatFixConstPtr &msg)
{
  // Once reference is initialized, we don't care about the GPS position
  if (start_point_.IsValid()) {
    return;
  }

  // Convert first received GPS position to ECEF and store as
  // reference coordinates for ENU
  igvc_flatland::WaypointList waypoint_msg;

  double ref_lat_rad_ = M_PI / 180.0 * msg->latitude;
  double ref_lon_rad_ = M_PI / 180.0 * msg->longitude;

  double s_lat = sin(ref_lat_rad_);
  double c_lat = cos(ref_lat_rad_);
  double s_lon = sin(ref_lon_rad_);
  double c_lon = cos(ref_lon_rad_);

  const double WGS84_A = 6378137.0;
  const double WGS84_E2 = 0.0066943799831668;
  double n = WGS84_A / sqrt(1.0 - WGS84_E2 * s_lat * s_lat);

  double ref_ecef_x_ = n * c_lat * c_lon;
  double ref_ecef_y_ = n * c_lat * s_lon;
  double ref_ecef_z_ = n * (1.0 - WGS84_E2) * s_lat;

  // Latch corresponding model position as an ENU offset
  start_point_ = base_body_->physics_body_->GetPosition();

  // Compute geodetic coordinates of the waypoints and publish them
  // on the latched "waypoints" topic.
  for (WaypointDict::iterator it = waypoints_.begin(); it != waypoints_.end(); ++it) {
    double dx = it->second.map_coords.x - start_point_.x;
    double dy = it->second.map_coords.y - start_point_.y;

    double ecef_x = ref_ecef_x_ - s_lon * dx - s_lat * c_lon * dy;
    double ecef_y = ref_ecef_y_ + c_lon * dx - s_lat * s_lon * dy;
    double ecef_z = ref_ecef_z_ + c_lat * dy;

    /* Convert ECEF to lat/lon */
    // Longitude is easy
    igvc_flatland::Waypoint wp;

    wp.longitude = atan2(ecef_y, ecef_x) * 180.0 * M_1_PI;

    // Iterative solution for latitude
    double r;
    double alt;
    double p = sqrt(ecef_x * ecef_x + ecef_y * ecef_y);
    double lat_rad = atan(p / ecef_z);
    for (int i = 0; i < 4; i++) {
      double s_lat = sin(lat_rad);
      r = WGS84_A / sqrt(1.0 - WGS84_E2 * s_lat * s_lat);
      alt = p / cos(lat_rad) - r;
      lat_rad = atan(ecef_z / p / (1 - WGS84_E2 * r / (r + alt)));
    }
    wp.latitude = lat_rad * 180.0 * M_1_PI;
    wp.name = it->first;
    waypoint_msg.waypoints.push_back(wp);
  }
  pub_waypoints_.publish(waypoint_msg);
}

void AutoJudge::markerTimerCb(const ros::TimerEvent &event)
{
  // Populate markers to show waypoints in Rviz
  visualization_msgs::Marker disk_marker;
  disk_marker.header.frame_id = "map";
  disk_marker.header.stamp = event.current_real;
  disk_marker.ns = "waypoint_disks";
  disk_marker.id = 0;
  disk_marker.type = visualization_msgs::Marker::CYLINDER;
  disk_marker.action = visualization_msgs::Marker::ADD;
  disk_marker.scale.z = 0.02;
  disk_marker.color.a = 0.2;
  visualization_msgs::Marker name_marker;
  name_marker.header.frame_id = "map";
  name_marker.header.stamp = event.current_real;
  name_marker.ns = "names";
  name_marker.id = 0;
  name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  name_marker.action = visualization_msgs::Marker::ADD;
  name_marker.scale.z = 1.0;
  name_marker.color.a = 0.8;
  name_marker.color.r = 1.0;
  name_marker.color.g = 1.0;
  name_marker.color.b = 1.0;

  visualization_msgs::MarkerArray waypoint_markers;
  for (std::map<std::string, WaypointStruct>::iterator it = waypoints_.begin(); it != waypoints_.end(); ++it) {
    disk_marker.pose.position.x = it->second.map_coords.x;
    disk_marker.pose.position.y = it->second.map_coords.y;
    disk_marker.pose.orientation.w = 1;
    if (it->second.hit) {
      disk_marker.color.r = 0.0;
      disk_marker.color.g = 0.9;
      disk_marker.color.b = 0.0;
    } else {
      disk_marker.color.r = 0.9;
      disk_marker.color.g = 0.0;
      disk_marker.color.b = 0.9;
    }
    disk_marker.scale.x = disk_marker.scale.y = 2.0 * it->second.dist_thres;

    name_marker.pose.position.x = disk_marker.pose.position.x;
    name_marker.pose.position.y = disk_marker.pose.position.y + 1.5;
    name_marker.text = it->first;
    waypoint_markers.markers.push_back(disk_marker);
    waypoint_markers.markers.push_back(name_marker);
    disk_marker.id++;
    name_marker.id++;
  }
  pub_waypoint_markers_.publish(waypoint_markers);
}

bool AutoJudge::AllWaypointsHit()
{
  for (WaypointDict::iterator it = waypoints_.begin(); it != waypoints_.end(); ++it) {
    if (!it->second.hit) {
      return false;
    }
  }
  return true;
}

int AutoJudge::CountMissedWaypoints()
{
  int result = 0;
  for (WaypointDict::iterator it = waypoints_.begin(); it != waypoints_.end(); ++it) {
    if (!it->second.hit) {
      result++;
    }
  }
  return result;
}

int AutoJudge::CountHitWaypoints()
{
  int result = 0;
  for (WaypointDict::iterator it = waypoints_.begin(); it != waypoints_.end(); ++it) {
    if (it->second.hit) {
      result++;
    }
  }
  return result;
}

void AutoJudge::LoadConfig(const YAML::Node &config)
{
  YamlReader reader(config);

  boost::filesystem::path map_path(reader.Get<std::string>("lines_map", ""));

  // Extract path to world YAML and use it to get absolute path to lines layer map
  boost::filesystem::path world_yaml_dir = boost::filesystem::path(ros::package::getPath("igvc_flatland"));
  if (map_path.string().front() != '/' && map_path.string().length() > 0) {
    map_path = world_yaml_dir / map_path;
  }

  // Load lines layer map YAML, extract its parameters, and load the map image for crossing detection
  YamlReader map_reader(map_path.string());
  resolution_ = map_reader.Get<double>("resolution");

  boost::filesystem::path image_path(map_reader.Get<std::string>("image"));
  if (image_path.string().front() != '/') {
    image_path =
      boost::filesystem::path(map_path).parent_path() / image_path;
  }

  cv::Mat map = cv::imread(image_path.string(), CV_LOAD_IMAGE_GRAYSCALE);
  if (map.empty()) {
    throw YAMLException("Failed to load " + Q(image_path.string()));
  }

  map.convertTo(lines_image_, CV_32FC1, 1.0 / 255.0);

  // Compute world coordinates of (0, 0) pixel in lines image
  Pose origin = map_reader.GetPose("origin");
  x0_ = origin.x;
  y0_ = origin.y + resolution_ * lines_image_.rows;

  // Read time limit parameter, defaults to six minutes
  time_limit_ = reader.Get<double>("time_limit", 360.0);

  // Read waypoints
  YamlReader waypoints_reader = reader.Subnode("waypoints", YamlReader::LIST);
  for (int i = 0; i < waypoints_reader.NodeSize(); i++) {
    YamlReader reader = waypoints_reader.Subnode(i, YamlReader::MAP);
    std::stringstream ss;
    ss << "waypoint_" << i + 1;
    std::string name = reader.Get<std::string>("name", ss.str());
    WaypointStruct wp;
    wp.dist_thres = reader.Get<double>("dist_thres", 1.0);
    flatland_server::Vec2 world_coords = reader.GetVec2("world_coords");
    wp.map_coords.Set(world_coords.x, world_coords.y);
    wp.hit = false;
    waypoints_[name] = wp;
  }

}

}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::AutoJudge, flatland_server::ModelPlugin)
