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
#include <move_base_msgs/MoveBaseAction.h>
#include <scitos_msgs/MoveBasePathAction.h>
#include <scitos_msgs/MoveBaseWallFollowAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

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
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <angles/angles.h>
#endif

#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;
typedef actionlib::SimpleActionServer<scitos_msgs::MoveBasePathAction> PathActionServer;
typedef actionlib::SimpleActionServer<scitos_msgs::MoveBaseWallFollowAction> WallFollowActionServer;

class ScitosDrive: public ScitosModule {
public:
	static ScitosModule*  Create() {
		return new ScitosDrive();
	}

	void initialize();

	template <typename Reflector> void reflect(Reflector& r);
	int startApplication(void);
	int startApplicationWithoutCleaning(void);
	int stopApplication(void);
	void application_status_callback(const std_msgs::Int32::ConstPtr& msg);

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
	void publish_grid_map(const mira::maps::OccupancyGrid& data, const ros::Publisher& pub, const std::string& frame_id);
	void cost_map_data_callback(mira::ChannelRead<mira::maps::GridMap<double,1> > data);
	void getCurrentRobotSpeed(double& robot_speed_x, double& robot_speed_theta);
	float computeFootprintToObstacleDistance(const mira::Pose2& target_pose);
	float computeFootprintToObstacleDistance(const mira::Pose2& target_pose, mira::Pose2& target_pose_in_merged_map, mira::maps::OccupancyGrid& merged_map,
			mira::maps::GridMap<float>& distance_transformed_map, boost::shared_ptr<mira::RigidTransform2f>& odometry_to_map, bool debug_texts=false);
	int waitForTargetApproach(const mira::Pose3& target_pose, const float goal_position_tolerance, const float goal_angle_tolerance, const float cost_map_threshold=-1);
	void publishComputedTarget(const tf::StampedTransform& transform);
	void publishCommandedTarget(const tf::StampedTransform& transform);
	ros::Publisher computed_trajectory_pub_;		// publishes the commanded targets for the robot trajectory
	ros::Publisher commanded_trajectory_pub_;		// publishes the commanded targets for the robot trajectory
	std::string map_frame_;			// name of the map coordinate system
	std::string map_clean_frame_;		// name of the map-clean coordinate system
	std::string map_segmented_frame_;	// name of the map-segmented coordinate system
	std::string robot_frame_;		// name of the robot base frame
	double robot_radius_;		// the radius of the inner circle of the bounding box of the robot footprint, in [m]
	double coverage_radius_;		// the radius of the area of the coverage device (cleaner, camera, ...), in [m]
	mira::RigidTransform3d coverage_offset_;		// the center offset of the area of the coverage device (cleaner, camera, ...) against robot_frame_, in [m]
	cv::Point2d map_world_offset_;	// in [m]
	double map_resolution_;		// in [m/cell]
	mira::Channel<mira::maps::OccupancyGrid> merged_map_channel_;
	mira::Channel<int> application_status_channel_;		// todo: hack: put to separate module
	ros::Subscriber application_status_sub_;		// todo: hack: put to separate module
	mira::Footprint footprint_;
	mira::model::CollisionTest collision_test_;
#endif

	boost::shared_ptr<MoveBaseActionServer> move_base_action_server_; ///< Action server which accepts requests for move base
	void move_base_callback(const move_base_msgs::MoveBaseGoalConstPtr& goal);

	boost::shared_ptr<PathActionServer> path_action_server_; ///< Action server which accepts requests for a path to follow
	void path_callback(const scitos_msgs::MoveBasePathGoalConstPtr& path);

	//boost::shared_ptr<PathActionServer> wall_follow_action_server_; ///< Action server which accepts requests for wall following
	//void wall_follow_callback(const scitos_msgs::MoveBasePathGoalConstPtr& path);
	boost::shared_ptr<WallFollowActionServer> wall_follow_action_server_; ///< Action server which accepts requests for wall following
	void wall_follow_callback(const scitos_msgs::MoveBaseWallFollowGoalConstPtr& path);

	double normalize_angle(double delta_angle);

	ros::Subscriber cmd_vel_subscriber_;
	ros::Publisher map_pub_;
	ros::Publisher map_clean_pub_;
	ros::Publisher map_segmented_pub_;
	ros::Publisher odometry_pub_;
	ros::Publisher bumper_pub_;
	ros::Publisher mileage_pub_;
	ros::Publisher motorstatus_pub_;
	ros::Publisher rfid_pub_;
    ros::Publisher magnetic_barrier_pub_;
    ros::Publisher emergency_stop_pub_;

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

