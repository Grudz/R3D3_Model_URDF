#ifndef IGVC_FLATLAND_AUTO_JUDGE_H_
#define IGVC_FLATLAND_AUTO_JUDGE_H_

#include <ros/package.h>
#include <Box2D/Box2D.h>
#include <opencv2/opencv.hpp>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <igvc_flatland/WaypointList.h>

namespace flatland_plugins
{

// State machine state enums
typedef enum {NO_CROSSING=0, LEFT_CROSSED, RIGHT_CROSSED, EXITED_COURSE} LineCrossingState;
typedef enum {LANE_1=0, NO_MANS_LAND, LANE_2, DONE} RunPhaseState;

// Structure containing the complete state of a waypoint
typedef struct {
    b2Vec2 map_coords; // Coordinates of waypoint in Flatland global frame
    double dist_thres; // Radius of the circle robot must enter to count waypoint
    double latitude;   // Latitude of waypoint
    double longitude;  // Longitude of waypoint
    bool hit;          // Flag indicating if robot hit the particular waypoint
} WaypointStruct;

// Dictionary mapping waypoint names to their corresponding states
typedef std::map<std::string, WaypointStruct> WaypointDict;

class AutoJudge : public flatland_server::ModelPlugin
{
public:
    // Flatland server callbacks to implement plugin behavior
    void OnInitialize ( const YAML::Node &config ) override;
    void BeforePhysicsStep ( const flatland_server::Timekeeper &timekeeper ) override;

    // Load parameters specified in the YAML file
    void LoadConfig ( const YAML::Node &config );

    // Receive the GPS fix being published by the GPS plugin. This is used to determine
    // the geodetic coordinates of the waypoints
    void recvFix ( const sensor_msgs::NavSatFixConstPtr &msg );

    // Receive the twist command messages to start automatic judging when the first is received
    void recvCmd ( const geometry_msgs::TwistConstPtr &msg );

    // ROS timer callback to refresh waypoint markers
    void markerTimerCb ( const ros::TimerEvent &event );

    // State machine for detecting reckless driving penalties and end-of-run lane departures
    void IterateLineCrossingDetection ( const flatland_server::Timekeeper &timekeeper );

    // Called after a run ending condition. Shows the score in the Flatland terminal depending on
    // the circumstances of the run being terminated.
    void DisplayScore();

    // Check status of wheels contacting lines and return true if a wheel
    // transitions from not being on a line
    bool CheckLeftLineCrossing();
    bool CheckRightLineCrossing();

    // See if a wheel enters the distance threshold of a waypoint and update
    // the waypoints_hit_ property
    void CheckWheelWaypointHit ( flatland_server::Body *body );

    // Returns true if all waypoints are hit as modified by CheckWheelWaypointHit
    bool AllWaypointsHit();

    // Returns number of waypoints not hit
    int CountMissedWaypoints();

    // Returns number of waypoints hit
    int CountHitWaypoints();

    // Look up a body's position on the lines image and return if it is on a line pixel
    bool GetLineStatus ( flatland_server::Body *body );

    // Callback function for the initiation of a Box2D physics collision
    void BeginContact ( b2Contact *contact ) override;

    // ROS interaction objects
    ros::Subscriber sub_fix_;
    ros::Subscriber sub_cmd_;
    ros::Publisher pub_waypoints_;
    ros::Publisher pub_waypoint_markers_;
    ros::Timer marker_timer_;

    cv::Mat lines_image_; // Locally-loaded copy of the lines layer image for detecting crossings
    double x0_;           // Global frame x coordinate corresponding to upper left corner of lines image
    double y0_;           // Global frame y coordinate corresponding to upper left corner of lines image
    double resolution_;   // Spatial resolution of lines image (m / px)

    // Pointers to physics bodies of the Flatland simulator. Used to extract
    // the true position of the bodies in the simulator's global frame.
    flatland_server::Body *base_body_;
    flatland_server::Body *left_wheel_body_;
    flatland_server::Body *right_wheel_body_;

    // Flags indicating if each wheel was on a line on the previous sample. This is used
    // to detect transitions from not being on a line.
    bool left_wheel_on_line_;
    bool right_wheel_on_line_;

    LineCrossingState line_crossing_state_; // Current state of the line crossing state machine
    double delay_;            // Time delay variable used in the line crossing state machine

    bool run_active_;         // True until a run end condition is triggered
    RunPhaseState run_phase_; // Curret state of the run
    bool collision_;          // Set to true when robot collides with something on the objects layer
    double run_time_;         // Current run duration
    double time_limit_;       // Total amount of time allowed to complete course
    double no_progress_time_; // Amount of time since progress was detected
    double distance_penalty_; // Amount of distance penalties accumulated during run

    // Variables for measuring distance of run and detecting and ignoring negative progress
    b2Vec2 dist_ref_;
    b2Vec2 last_dist_ref_;
    double latched_dist_;
    double current_dist_2_;
    bool going_backward_;    // Flag indicating that the robot is making negative progress
    double lane_1_dist_;     // Approximate length of the first lined section of the course

    WaypointDict waypoints_;      // Dictionatry of waypoints read from YAML
    bool at_start_;               // True if within 2 meters of start.
    bool received_cmd_;           // Set to true after receiving first twist command message
    b2Vec2 start_point_;          // Coordinates of starting point
    double start_heading_;        // Starting heading
    std::string first_opening_;   // Name of the waypoint marking the first opening to no man's land
    std::string second_opening_;  // Name of the waypoint marking the exit from no man's land
};

}


#endif // IGVC_FLATLAND_AUTO_JUDGE_H_
