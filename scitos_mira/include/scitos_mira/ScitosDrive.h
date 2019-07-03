/* ScitosDrive.h
 *      Author: chris burbridge
 *
 * Module for interfacing all related to drive: odometry, motor controller state etc etc
 * http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#Drive_Section
 */

#ifndef SCITOSBASE_H
#define SCITOSBASE_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <robot/Odometry.h> //# MIRA odometry
#include <robot/RobotModel.h> //# MIRA robot model Package(RobotDataTypes)
#include "scitos_mira/ScitosModule.h"
#include <std_msgs/Bool.h>
#include <scitos_msgs/ResetMotorStop.h>
#include <scitos_msgs/ResetOdometry.h>
#include <scitos_msgs/ResetBarrierStop.h>
#include <scitos_msgs/EmergencyStop.h>
#include <scitos_msgs/EnableMotors.h>
#include <scitos_msgs/MotorStatus.h>
#include <scitos_msgs/ChangeForce.h>
#include <scitos_msgs/EnableRfid.h>
#include <scitos_msgs/BarrierStatus.h>
#include <utils/Time.h>

#include <actionlib/server/simple_action_server.h>
#include <scitos_msgs/MoveBaseAction.h>
#include <scitos_msgs/MoveBasePathAction.h>
#include <scitos_msgs/MoveBaseWallFollowAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <model/CollisionTest.h>

//includes for move_base
#ifdef __WITH_PILOT__
#include <pilot/SmoothTransitionTask.h>
#include <pilot/WallDistanceTask.h>
#include <navigation/Task.h>
#include <navigation/tasks/PositionTask.h>
#include <navigation/tasks/OrientationTask.h>
#include <navigation/tasks/VelocityTask.h>
#include <navigation/tasks/PreferredDirectionTask.h>
#include <navigation/tasks/PathFollowTask.h>
#include <maps/OccupancyGrid.h> //# MIRA occupancy grid map
#include <geometry/Point.h> // used to display custom detections
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <angles/angles.h>
#include <serialization/adapters/pcl/point_types.h>	// todo:PCL -> uncomment
#include <serialization/adapters/pcl/point_cloud.h>	// todo:PCL -> uncomment
#include <pcl_conversions/pcl_conversions.h>
#endif

#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>

typedef actionlib::SimpleActionServer<scitos_msgs::MoveBaseAction> MoveBaseActionServer;
typedef actionlib::SimpleActionServer<scitos_msgs::MoveBasePathAction> PathActionServer;
typedef actionlib::SimpleActionServer<scitos_msgs::MoveBaseWallFollowAction> WallFollowActionServer;

enum TargetCode { TARGET_GOAL_REACHED = 0, TARGET_INACESSIBLE = 1, TARGET_PILOT_NOT_IN_PLAN_AND_DRIVE_MODE = 2, TARGET_ROBOT_DOES_NOT_MOVE = 3};

class ScitosDrive: public ScitosModule {
public:

	enum ActionServerType { PATH_ACTION, WALL_FOLLOW_ACTION,  MOVE_BASE_ACTION };

	static ScitosModule*  Create() {
		return new ScitosDrive();
	}

	void initialize();

	void velocity_command_callback(const geometry_msgs::Twist::ConstPtr& msg);

	void odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data);

	void bumper_data_callback(mira::ChannelRead<bool> data);
	void mileage_data_callback(mira::ChannelRead<float> data);
	void motor_status_callback(mira::ChannelRead<uint8> data);
	void rfid_status_callback(mira::ChannelRead<uint64> data);
	void nav_pilot_event_status_callback(mira::ChannelRead<std::string> data);

	bool reset_motor_stop(scitos_msgs::ResetMotorStop::Request  &req, scitos_msgs::ResetMotorStop::Response &res);
	bool reset_odometry(scitos_msgs::ResetOdometry::Request  &req, scitos_msgs::ResetOdometry::Response &res);
	bool emergency_stop(scitos_msgs::EmergencyStop::Request  &req, scitos_msgs::EmergencyStop::Response &res);
	bool enable_motors(scitos_msgs::EnableMotors::Request  &req, scitos_msgs::EnableMotors::Response &res);
	bool change_force(scitos_msgs::ChangeForce::Request  &req, scitos_msgs::ChangeForce::Response &res);
	bool enable_rfid(scitos_msgs::EnableRfid::Request  &req, scitos_msgs::EnableRfid::Response &res);
	bool reset_barrier_stop(scitos_msgs::ResetBarrierStop::Request  &req, scitos_msgs::ResetBarrierStop::Response &res);
	void publish_barrier_status();

private:
	ScitosDrive();

#ifdef __WITH_PILOT__
	void writeParametersToROSParamServer();
	void map_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data);
	void map_clean_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data);
	void map_segmented_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data);
	void camera2_pcl_data_callback(mira::ChannelRead<pcl::PointCloud<pcl::PointXYZRGB> > data);
	void publishCameraPosition(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg);
	void publish_grid_map(const mira::maps::OccupancyGrid& data, const ros::Publisher& pub, const std::string& frame_id);
	void cost_map_data_callback(mira::ChannelRead<mira::maps::GridMap<double,1> > data);
	void getCurrentRobotSpeed(double& robot_speed_x, double& robot_speed_theta);
	float computeFootprintToObstacleDistance(const mira::Pose2& target_pose);
	float computeFootprintToObstacleDistance(const mira::Pose2& target_pose, mira::Pose2& target_pose_in_merged_map, mira::maps::OccupancyGrid& merged_map,
			mira::maps::GridMap<float>& distance_transformed_map, boost::shared_ptr<mira::RigidTransform2f>& odometry_to_map, bool debug_texts=false);

	bool isActionServerPreemptRequested(ScitosDrive::ActionServerType action_server_type) const;
	mira::Pose3 getRobotPose() const;
	double computeEuclideanDistanceToGoal(const mira::Pose3& pose_a, const mira::Pose3& pose_b) const;
	void stopRobotAtCurrentPosition();
	TargetCode setTaskAndWaitForTarget(const mira::Pose3 target, float position_accuracy, float position_tolerance, float angle_accuracy, float angle_tolerance,
			ScitosDrive::ActionServerType action_server_type, float cost_map_threshold, double target_wall_distance=-1);
	TargetCode waitForTargetApproach(const mira::Pose3& target_pose, float goal_position_tolerance, float goal_angle_tolerance, ScitosDrive::ActionServerType action_server_type, float cost_map_threshold=-1);

	void publishComputedTarget(const tf::StampedTransform& transform);
	void publishCommandedTarget(const tf::StampedTransform& transform);
	ros::Publisher computed_trajectory_pub_;
	ros::Publisher commanded_trajectory_pub_;

	const std::string map_frame_ { "map" };
	const std::string map_clean_frame_; // todo (rmb-ma) not used
	const std::string map_segmented_frame_ { "map_segmented" }; // TODO don't use the same frame than map_frame_
	const std::string robot_frame_ { "base_link" };
	const std::string camera2_frame_ { "camera2_optical_frame" };

    double robot_radius_;		// the radius of the inner circle of the bounding box of the robot footprint, in [m]
	double coverage_radius_;		// the radius of the area of the coverage device (cleaner, camera, ...), in [m]
	mira::RigidTransform3d coverage_offset_;		// the center offset of the area of the coverage device (cleaner, camera, ...) against robot_frame_, in [m]
	cv::Point2d map_world_offset_;	// in [m]
	double map_resolution_;		// in [m/cell]
	mira::maps::OccupancyGrid map_;
	mira::Channel<mira::maps::OccupancyGrid> merged_map_channel_;
	mira::Footprint footprint_;
	mira::model::CollisionTest collision_test_;

#endif

	boost::shared_ptr<MoveBaseActionServer> move_base_action_server_; ///< Action server which accepts requests for move base
	void move_base_callback(const scitos_msgs::MoveBaseGoalConstPtr& goal);

	boost::shared_ptr<PathActionServer> path_action_server_; ///< Action server which accepts requests for a path to follow
	mira::Pose2 computeRightCandidate(const mira::Pose2 &target_pose_in_merged_map, const double offset, cv::Point2d direction_left, const mira::maps::OccupancyGrid &merged_map,
			const mira::RigidTransform2f &map_to_odometry, const cv::Point2d &map_world_offset_, double map_resolution_,
			const double min_obstacle_distance, const cv::Mat &area_map,
			const mira::maps::GridMap<float> &distance_transformed_map, bool &found) const;
	mira::Pose2 computeLeftCandidate(const mira::Pose2 &target_pose_in_merged_map, const double offset, cv::Point2d direction_left, const mira::maps::OccupancyGrid &merged_map,
			const mira::RigidTransform2f &map_to_odometry, const cv::Point2d &map_world_offset_, double map_resolution_,
			const double min_obstacle_distance, const cv::Mat &area_map,
			const mira::maps::GridMap<float> &distance_transformed_map, bool &found) const;
	bool computeAlternativeTargetIfNeeded(mira::Pose3 &target_pose, const double next_x, const double next_y, const double min_obstacle_distance, const cv::Mat& area_map);
	void path_callback(const scitos_msgs::MoveBasePathGoalConstPtr& path);

	void displayWallFollowerPath(const std::vector<cv::Vec3d>& wall_poses, const cv::Mat& area_map, double map_resolution, const cv::Point& map_origin) const;
	bool computeClosestPos(const cv::Mat& level_set_map, const cv::Point& current_pos, cv::Point& best_pos) const;
	bool computePosInNeighborhoodWithMaxCosinus(cv::Mat& level_set_map, const cv::Point& current_pos, cv::Point& next_pos, const cv::Mat& driving_direction) const;
	void computeWallPosesDense(const scitos_msgs::MoveBaseWallFollowGoalConstPtr& goal, std::vector<cv::Vec3d>& wall_poses_dense) const;

	boost::shared_ptr<WallFollowActionServer> wall_follow_action_server_; ///< Action server which accepts requests for wall following
	void wall_follow_callback(const scitos_msgs::MoveBaseWallFollowGoalConstPtr& path);

	void map_msgToCvFormat(const sensor_msgs::Image& image_map, cv::Mat& map) const;
	double normalize_angle(double delta_angle) const;

	ros::Subscriber cmd_vel_subscriber_;
	ros::Publisher map_pub_;
	ros::Publisher map_clean_pub_;
	ros::Publisher map_segmented_pub_;
	ros::Publisher camera2_pcl_pub;
	ros::Publisher odometry_pub_;
	ros::Publisher bumper_pub_;
	ros::Publisher mileage_pub_;
	ros::Publisher motorstatus_pub_;
	ros::Publisher rfid_pub_;
	ros::Publisher magnetic_barrier_pub_;
	ros::Publisher emergency_stop_pub_;
	ros::Subscriber camera_depth_points_sub_;
	ros::Publisher camera2_pcl_pub_;

	ros::ServiceServer reset_motor_stop_service_;
	ros::ServiceServer reset_odometry_service_;
	ros::ServiceServer emergency_stop_service_;
	ros::ServiceServer enable_motors_service_;
	ros::ServiceServer change_force_service_;
	ros::ServiceServer enable_rfid_service_;
    ros::ServiceServer reset_barrier_stop_service_;
    std_msgs::Bool emergency_stop_;
    scitos_msgs::BarrierStatus barrier_status_;

    std::string nav_pilot_event_status_;
    boost::mutex nav_pilot_event_status_mutex_;

    nav_msgs::Odometry odom_msg_;
    boost::mutex odom_msg_mutex_;

#ifdef __WITH_PILOT__
    mira::maps::GridMap<double,1> cost_map_;
    boost::mutex cost_map_mutex_;
#endif
};

#endif

