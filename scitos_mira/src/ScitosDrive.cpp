#include "scitos_mira/ScitosDrive.h"
#include "scitos_mira/ScitosG5.h"

#include <transform/RigidTransform.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt64.h>
#include <geometry/Point.h>
#include <maps/PointCloudFormat.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <fstream> // DEBUG PURPOSE

#define PI 3.14159265359

uint64 MAGNETIC_BARRIER_RFID_CODE=0xabababab;

ScitosDrive::ScitosDrive() : ScitosModule(std::string ("Drive")) {
}

void ScitosDrive::initialize() {
  odometry_pub_ = robot_->getRosNode().advertise<nav_msgs::Odometry>("/odom", 20);
  bumper_pub_ = robot_->getRosNode().advertise<std_msgs::Bool>("/bumper", 20);
  mileage_pub_ = robot_->getRosNode().advertise<std_msgs::Float32>("/mileage", 20);
  motorstatus_pub_ = robot_->getRosNode().advertise<scitos_msgs::MotorStatus>("/motor_status", 20);
  rfid_pub_ = robot_->getRosNode().advertise<std_msgs::UInt64>("/rfid", 20);
  magnetic_barrier_pub_ = robot_->getRosNode().advertise<scitos_msgs::BarrierStatus>("/barrier_status", 20);
  emergency_stop_pub_ = robot_->getRosNode().advertise<std_msgs::Bool>("/emergency_stop_status", 20, true);

  robot_->getMiraAuthority().subscribe<mira::robot::Odometry2>("/robot/Odometry",
							       &ScitosDrive::odometry_data_callback, this);
  robot_->getMiraAuthority().subscribe<bool>("/robot/Bumper",
					     &ScitosDrive::bumper_data_callback, this);
  robot_->getMiraAuthority().subscribe<float>("/robot/Mileage",
					      &ScitosDrive::mileage_data_callback, this);
  robot_->getMiraAuthority().subscribe<uint8>("/robot/MotorStatus",
					      &ScitosDrive::motor_status_callback, this);
  robot_->getMiraAuthority().subscribe<uint64>("/robot/RFIDFloorTag",
					      &ScitosDrive::rfid_status_callback, this);

#ifdef __WITH_PILOT__
  // Pilot
  robot_->getMiraAuthority().subscribe<std::string>("/navigation/PilotEvent", &ScitosDrive::nav_pilot_event_status_callback, this);

  // maps
  map_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>("map_original", 1, true);
  map_clean_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>(map_frame_, 1, true);
  map_segmented_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>(map_segmented_frame_, 1, true);
  camera2_pcl_pub_ = robot_->getRosNode().advertise<sensor_msgs::PointCloud2>("camera2_pcl", 1, true);

  mira::Channel<mira::maps::OccupancyGrid> map_channel = robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/static/Map", &ScitosDrive::map_data_callback, this);
  robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/cleaning/Map", &ScitosDrive::map_clean_data_callback, this);	// todo: hack:
  robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/segmentation/Map", &ScitosDrive::map_segmented_data_callback, this);

  // todo:PCL -> uncomment
  robot_->getMiraAuthority().subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/robot/depthCam2/pcl/PCLOut", &ScitosDrive::camera2_pcl_data_callback, this);
  camera_depth_points_sub_ = robot_->getRosNode().subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1,
		  &ScitosDrive::publishCameraPosition, this);

  mira::Channel<mira::maps::GridMap<double,1> > cost_map_channel = robot_->getMiraAuthority().subscribe<mira::maps::GridMap<double,1> >("/maps/cost/PlannerMap_FinalCostMap", &ScitosDrive::cost_map_data_callback, this);
  robot_->getMiraAuthority().bootup("Waiting for data on channel /maps/cost/PlannerMap_FinalCostMap");
  cost_map_channel.waitForData();
  merged_map_channel_ = robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid >("ObstacleMap");

  computed_trajectory_pub_ = robot_->getRosNode().advertise<geometry_msgs::TransformStamped>("/room_exploration/coverage_monitor_server/computed_target_trajectory_monitor", 1);
  commanded_trajectory_pub_ = robot_->getRosNode().advertise<geometry_msgs::TransformStamped>("/room_exploration/coverage_monitor_server/commanded_target_trajectory_monitor", 1);

  // read in robot radius, coverage radius, and coverage area center offset against robot base link
  writeParametersToROSParamServer();

  if (!footprint_.empty()) {
	  //mira::ChannelRead<mira::maps::OccupancyGrid> merged_map = robot_->getMiraAuthority().waitForData(merged_map_channel_, mira::Duration::seconds(10));
	  mira::ChannelRead<mira::maps::OccupancyGrid> map = robot_->getMiraAuthority().waitForData(map_channel, mira::Duration::seconds(10));
	  if (!map.isValid())
		  ROS_ERROR("Cannot read map channel '%s'", map_channel.getID().c_str());
	  else
		  collision_test_.initialize(footprint_, map->getCellSize());
  }

#endif

  cmd_vel_subscriber_ = robot_->getRosNode().subscribe("/cmd_vel", 1000, &ScitosDrive::velocity_command_callback, this);
  // mira navigation data types: http://www.mira-project.org/MIRA-doc/domains/robot/SCITOSConfigs/index.html
  move_base_action_server_ = boost::shared_ptr<MoveBaseActionServer>(new MoveBaseActionServer(robot_->getRosNode(), "/move_base", boost::bind(&ScitosDrive::move_base_callback, this, _1), false)); // this initializes the action server; important: always set the last parameter to false
  move_base_action_server_->start();

  path_action_server_ = boost::shared_ptr<PathActionServer>(new PathActionServer(robot_->getRosNode(), "/move_base_path", boost::bind(&ScitosDrive::path_callback, this, _1), false)); // this initializes the action server; important: always set the last parameter to false
  path_action_server_->start();

  //wall_follow_action_server_ = boost::shared_ptr<PathActionServer>(new PathActionServer(robot_->getRosNode(), "/move_base_wall_follow", boost::bind(&ScitosDrive::wall_follow_callback, this, _1), false)); // this initializes the action server; important: always set the last parameter to false
  wall_follow_action_server_ = boost::shared_ptr<WallFollowActionServer>(new WallFollowActionServer(robot_->getRosNode(), "/move_base_wall_follow", boost::bind(&ScitosDrive::wall_follow_callback, this, _1), false)); // this initializes the action server; important: always set the last parameter to false
  wall_follow_action_server_->start();

  reset_motor_stop_service_ = robot_->getRosNode().advertiseService("/reset_motorstop", &ScitosDrive::reset_motor_stop, this);
  reset_odometry_service_ = robot_->getRosNode().advertiseService("/reset_odometry", &ScitosDrive::reset_odometry, this);
  emergency_stop_service_ = robot_->getRosNode().advertiseService("/emergency_stop", &ScitosDrive::emergency_stop, this);
  enable_motors_service_ = robot_->getRosNode().advertiseService("/enable_motors", &ScitosDrive::enable_motors, this);
  change_force_service_ = robot_->getRosNode().advertiseService("/change_force", &ScitosDrive::change_force, this);
  enable_rfid_service_ = robot_->getRosNode().advertiseService("/enable_rfid", &ScitosDrive::enable_rfid, this);
  reset_barrier_stop_service_ = robot_->getRosNode().advertiseService("/reset_barrier_stop", &ScitosDrive::reset_barrier_stop, this);

  bool magnetic_barrier_enabled = true;
  ros::param::param("~magnetic_barrier_enabled", magnetic_barrier_enabled, magnetic_barrier_enabled);
  if (magnetic_barrier_enabled) {
    set_mira_param_("MainControlUnit.RearLaser.Enabled", "true");
  }  else {
    ROS_WARN("Magnetic barrier motor stop not enabled.");
    set_mira_param_("MainControlUnit.RearLaser.Enabled", "false");
  }

  emergency_stop_.data = false;
  barrier_status_.barrier_stopped = false;
  barrier_status_.last_detection_stamp = ros::Time(0);
  robot_->registerSpinFunction(boost::bind(&ScitosDrive::publish_barrier_status, this));

  robot_->getMiraAuthority().bootupFinished();
}

void ScitosDrive::velocity_command_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  if ( !barrier_status_.barrier_stopped && !emergency_stop_.data) {
      mira::RigidTransform<float, 2> speed(msg->linear.x, 0, msg->angular.z);
      mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", "setVelocity", speed);
      r.wait();
  }
}

void ScitosDrive::bumper_data_callback(mira::ChannelRead<bool> data) {
	std_msgs::Bool out;
	out.data=data->value();
	bumper_pub_.publish(out);
}

void ScitosDrive::mileage_data_callback(mira::ChannelRead<float> data) {
  std_msgs::Float32 out;
  out.data = data->value();
  mileage_pub_.publish(out);
}

void ScitosDrive::rfid_status_callback(mira::ChannelRead<uint64> data) {
  if (data->value() == MAGNETIC_BARRIER_RFID_CODE) {
    barrier_status_.barrier_stopped = true;
    barrier_status_.last_detection_stamp = ros::Time().fromNSec(data->timestamp.toUnixNS()); //Before it was ros::Time::now(). Changed it to the actual mira timestamp
  }
  std_msgs::UInt64 out;
  out.data = data->value();
  rfid_pub_.publish(out);
}


void ScitosDrive::motor_status_callback(mira::ChannelRead<uint8> data) {
  ros::Time time_now = ros::Time().fromNSec(data->timestamp.toUnixNS()); //Before it was ros::Time::now(). Changed it to the actual mira timestamp

  scitos_msgs::MotorStatus s;
  s.header.stamp=time_now;
  s.normal = (*data) & 1;
  s.motor_stopped = (*data) & (1 << 1);
  s.free_run = (*data) & (1 << 2);
  s.emergency_button_pressed = (*data) & (1 << 3);
  s.bumper_pressed = (*data) & (1 << 4);
  s.bus_error = (*data) & (1 << 5);
  s.stall_mode_flag = (*data) & (1 << 6);
  s.internal_error_flag = (*data) & (1 << 7);

  motorstatus_pub_.publish(s);
}

void ScitosDrive::publish_barrier_status() {
  barrier_status_.header.stamp = ros::Time::now();
  magnetic_barrier_pub_.publish(barrier_status_);
}

void ScitosDrive::odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data ) {
	/// new odometry data through mira; put it out in ros
	ros::Time odom_time = ros::Time().fromNSec(data->timestamp.toUnixNS()); //Before it was ros::Time::now(). Changed it to the actual mira timestamp
	geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(
			data->value().pose.phi());

	// Publish as a nav_msgs::Odometry
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = odom_time;
	odom_msg.header.frame_id = "/odom";
	odom_msg.child_frame_id = "/base_footprint";

	// set the position
	odom_msg.pose.pose.position.x = data->value().pose.x();
	odom_msg.pose.pose.position.y = data->value().pose.y();
	odom_msg.pose.pose.orientation = orientation;

	// set the velocity
	odom_msg.twist.twist.linear.x = data->value().velocity.x();
	odom_msg.twist.twist.angular.z = data->value().velocity.phi();

	{
		boost::mutex::scoped_lock lock(odom_msg_mutex_);
		odom_msg_ = odom_msg;
	}

	odometry_pub_.publish(odom_msg);

	// Publish a TF
	geometry_msgs::TransformStamped odom_tf;
	odom_tf.header.stamp = odom_time;
	odom_tf.header.frame_id = "/odom";
	odom_tf.child_frame_id = "/base_footprint";

	odom_tf.transform.translation.x = data->value().pose.x();
	odom_tf.transform.translation.y = data->value().pose.y();
	odom_tf.transform.translation.z = 0.0;
	odom_tf.transform.rotation = orientation;
	// send the transform
	robot_->getTFBroadcaster().sendTransform(odom_tf);

#ifdef __WITH_PILOT__
	// publish localization if available
	geometry_msgs::TransformStamped localization_tf;
	localization_tf.header.stamp = odom_time;
	localization_tf.header.frame_id = map_frame_;
	localization_tf.child_frame_id = robot_frame_;	//"/odom";
	//mira::RigidTransform3d map_to_odometry = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/robot/OdometryFrame", "/maps/MapFrame");
	mira::RigidTransform3d map_to_odometry = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/robot/RobotFrame", "/maps/MapFrame");
	tf::transformEigenToMsg(map_to_odometry, localization_tf.transform);
	// send the transform
	robot_->getTFBroadcaster().sendTransform(localization_tf);
#endif
}

#ifdef __WITH_PILOT__
void ScitosDrive::writeParametersToROSParamServer()
{
	// read in robot radius, coverage radius, and coverage area center offset against robot base link
	try
	{
		mira::RPCFuture<mira::robot::RobotModelPtr> result = robot_->getMiraAuthority().callService<mira::robot::RobotModelPtr>("/robot/Robot", "getRobotModel");
		bool success = result.timedWait(mira::Duration::seconds(10));

		if (success && result.hasValue())
		{
			mira::robot::RobotModelPtr robot_model = result.get();
			footprint_ = robot_model->getFootprint();
			mira::Footprint coverage_footprint = robot_model->getFootprint("", mira::Time::now(), "CleaningTool");
			robot_radius_ = footprint_.getInnerRadius();
			// todo: hack: only works for circle footprints
			coverage_radius_ = 0.5*(coverage_footprint.getOuterRadius() - coverage_footprint.getInnerRadius());
			coverage_offset_ = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/modules/brushcleaning/BrushFrame", "/robot/RobotFrame");
		}
		else
		{
			robot_radius_ = 0.325;
			coverage_radius_ = 0.25;
			coverage_offset_ = mira::RigidTransform3d(0.29, -0.114, 0.0, 0.0, 0.0, 0.0);
			std::cout << "Error: could not read robot parameters. Taking standard values." << std::endl;
		}
	}
	catch (std::exception& error)
	{
		std::cout << error.what() << std::endl;
	}

	std::cout << "########## Robot Configuration ##########" << std::endl;
	std::cout << "robot_radius=" << robot_radius_ << "   coverage_radius=" << coverage_radius_;
	std::cout << "   coverage_offset=(" << coverage_offset_.x() << ", " << coverage_offset_.y() << ")\n" << std::endl;

	// write parameters to ROS parameter server
	robot_->getRosNode().setParam("map_frame", map_frame_);
	robot_->getRosNode().setParam("robot_frame", robot_frame_);
	robot_->getRosNode().setParam("robot_radius", robot_radius_);
	robot_->getRosNode().setParam("coverage_radius", coverage_radius_);
	robot_->getRosNode().setParam("coverage_offset_x", coverage_offset_.x());
	robot_->getRosNode().setParam("coverage_offset_y", coverage_offset_.y());
}

void ScitosDrive::map_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data)
{
	// convert map to ROS format
	ROS_INFO("ScitosDrive::map_data_callback: Received map.");
	map_world_offset_ = cv::Point2d(data->getWorldOffset()[0], data->getWorldOffset()[1]);
	map_resolution_ = data->getCellSize();
	map_ = data->value();
	publish_grid_map(data->value(), map_pub_, map_frame_);
}

void ScitosDrive::map_clean_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data)
{
	ROS_INFO("ScitosDrive::map_clean_data_callback: Received map_clean.");
	publish_grid_map(data->value(), map_clean_pub_, map_frame_);	// todo: hack: using a separately edited map as the "real" map for planning
}

void ScitosDrive::map_segmented_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data)
{
	ROS_INFO("ScitosDrive::map_segmented_data_callback: Received map_segmented.");
	publish_grid_map(data->value(), map_segmented_pub_, map_segmented_frame_);
}

void ScitosDrive::publishCameraPosition(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg)
{
	ros::Time cam_pcl_time = ros::Time::now();

	geometry_msgs::TransformStamped localization_tf;
	localization_tf.header.stamp = cam_pcl_time;
	localization_tf.header.frame_id = robot_frame_;
	localization_tf.child_frame_id = camera2_frame_;
	//mira::RigidTransform3d map_to_odometry = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/robot/OdometryFrame", "/maps/MapFrame");
	mira::RigidTransform3d map_to_odometry = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/robot/depthCam2/PrimeSenseFrame", "/robot/RobotFrame");
	tf::transformEigenToMsg(map_to_odometry, localization_tf.transform);
	// send the transform
	robot_->getTFBroadcaster().sendTransform(localization_tf);
}

// todo: wmb: change to multiple camera support, add camera number boost:bind as fixed param
void ScitosDrive::camera2_pcl_data_callback(mira::ChannelRead<pcl::PointCloud<pcl::PointXYZRGB>> data)
{
	// todo:PCL -> uncomment
	ros::Time cam_pcl_time = ros::Time::now();
	//convert point cloud to ROS format
	ROS_INFO("ScitosDrive::camera2_pcl_data_callback: Received camera2_pcl.");
	sensor_msgs::PointCloud2 camera_pcl;
	pcl::toROSMsg(data->value(), camera_pcl);
	camera_pcl.header.stamp = cam_pcl_time;
	camera_pcl.header.frame_id = camera2_frame_;
	camera2_pcl_pub_.publish(camera_pcl);

	// publish localization if available
	geometry_msgs::TransformStamped localization_tf;
	localization_tf.header.stamp = cam_pcl_time;
	localization_tf.header.frame_id = robot_frame_;
	localization_tf.child_frame_id = camera2_frame_;
	//mira::RigidTransform3d map_to_odometry = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/robot/OdometryFrame", "/maps/MapFrame");
	mira::RigidTransform3d map_to_odometry = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/robot/depthCam2/PrimeSenseFrame", "/robot/RobotFrame");
	tf::transformEigenToMsg(map_to_odometry, localization_tf.transform);
	// send the transform
	robot_->getTFBroadcaster().sendTransform(localization_tf);
}

void ScitosDrive::publish_grid_map(const mira::maps::OccupancyGrid& data, const ros::Publisher& pub, const std::string& frame_id)
{
	// convert map to ROS format
	nav_msgs::OccupancyGrid grid_msg;
	grid_msg.header.stamp = ros::Time::now();
	grid_msg.header.frame_id = frame_id;
	grid_msg.info.resolution = data.getCellSize();
	grid_msg.info.width = data.width();
	grid_msg.info.height = data.height();
	grid_msg.info.origin.position.x = -data.getWorldOffset()[0];
	grid_msg.info.origin.position.y = -data.getWorldOffset()[1];

	cv::Mat map = data.getMat();
	grid_msg.data.resize(map.cols*map.rows);
	int i = 0;
	for (int v = 0; v < map.rows; ++v)
	{
		unsigned char* map_ptr = map.ptr(v);
		for (int u = 0; u < map.cols; ++u, ++i, ++map_ptr)
		{
			char value = (char)((double)*map_ptr*100./255);
			// invert values: 255 in MIRA is 100 in ROS = obstacle, 0 in MIRA is 0 in ROS = free space
			// further obey that accessible areas are any value <=100 in MIRA, i.e. <=39.2 in ROS
			value = (value < 5 ? 0 : value);
			value = (value > 95 ? 100 : value);
			grid_msg.data[i] = value;
		}
	}
	pub.publish(grid_msg);
}

void ScitosDrive::cost_map_data_callback(mira::ChannelRead<mira::maps::GridMap<double,1> > data)
{
	// todo: cost map is never used
	// store cost map
	boost::mutex::scoped_lock lock(cost_map_mutex_);
	cost_map_ = data->clone();
}

#endif

void ScitosDrive::nav_pilot_event_status_callback(mira::ChannelRead<std::string> data)
{
	boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);
	nav_pilot_event_status_ = data->value();
}

TargetCode ScitosDrive::setTaskAndWaitForTarget(const mira::Pose3 target, float position_accuracy, float position_tolerance, float angle_accuracy, float angle_tolerance,
		 ScitosDrive::ActionServerType action_server_type, float cost_map_threshold, double target_wall_distance)
{

	mira::navigation::TaskPtr task(new mira::navigation::Task());
	mira::Point2f target2f(target.x(), target.y());
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(target2f, position_accuracy, position_accuracy)));
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(target.yaw(), angle_accuracy)));
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::VelocityTask(mira::Velocity2(max_speed_x, 0.0, max_speed_phi))));	// limit the max allowed speed
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));
	//task->getSubTask<mira::navigation::PreferredDirectionTask>()->direction = mira::navigation::PreferredDirectionTask::BOTH; // only for testing purpose
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::SmoothTransitionTask(true, true)));

	if (target_wall_distance > 0)
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::WallDistanceTask(target_wall_distance, 1.0, mira::navigation::WallDistanceTask::KEEP_RIGHT)));

	mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
	r.wait();

	TargetCode return_code = waitForTargetApproach(target, position_tolerance, angle_tolerance, action_server_type, cost_map_threshold);
	if (return_code == TARGET_ROBOT_DOES_NOT_MOVE) {
		task->getSubTask<mira::navigation::PreferredDirectionTask>()->direction = mira::navigation::PreferredDirectionTask::BOTH;
		robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
		return_code = waitForTargetApproach(target, position_tolerance, angle_tolerance, action_server_type, cost_map_threshold);
	}
	return return_code;
}

void ScitosDrive::move_base_callback(const scitos_msgs::MoveBaseGoalConstPtr& goal)
{
#ifdef __WITH_PILOT__
	mira::Pose3 target_pose(Eigen::Vector3f(goal->target_pose.pose.position.x, goal->target_pose.pose.position.y, goal->target_pose.pose.position.z),
						Eigen::Quaternionf(goal->target_pose.pose.orientation.w, goal->target_pose.pose.orientation.x, goal->target_pose.pose.orientation.y, goal->target_pose.pose.orientation.z));

	const float angle_tolerance = goal->goal_angle_tolerance;
	const float position_tolerance = goal->goal_position_tolerance;
	const float cost_map_threshold = 0.f;
	const TargetCode return_code = setTaskAndWaitForTarget(target_pose, position_tolerance, position_tolerance, angle_tolerance, angle_tolerance, ScitosDrive::MOVE_BASE_ACTION, cost_map_threshold);

	MoveBaseActionServer::Result res;
	if (return_code == TARGET_GOAL_REACHED || return_code == TARGET_PILOT_NOT_IN_PLAN_AND_DRIVE_MODE)
		move_base_action_server_->setSucceeded(res);
	else
		move_base_action_server_->setAborted(res, "move_base_callback returned with status " + return_code);
#else
	ROS_ERROR("ScitosDrive::move_base_callback: This function is not compiled. Install the MIRA Pilot addon and make sure it is found by cmake.");

	// this sends the response back to the caller
	MoveBaseActionServer::Result res;
	move_base_action_server_->setAborted(res);
#endif
}

double ScitosDrive::normalize_angle(double delta_angle) const
{
	while (delta_angle < -PI)
		delta_angle += 2*PI;

	while (delta_angle > PI)
		delta_angle -= 2*PI;

	return delta_angle;
}

#ifdef __WITH_PILOT__
void ScitosDrive::getCurrentRobotSpeed(double& robot_speed_x, double& robot_speed_theta)
{
	boost::mutex::scoped_lock lock(odom_msg_mutex_);
	robot_speed_x = odom_msg_.twist.twist.linear.x;
	robot_speed_theta = odom_msg_.twist.twist.angular.z;
}

float ScitosDrive::computeFootprintToObstacleDistance(const mira::Pose2& target_pose)
{
	mira::Pose2 target_pose_in_merged_map;
	mira::maps::OccupancyGrid merged_map;
	mira::maps::GridMap<float> distance_transformed_map;
	boost::shared_ptr<mira::RigidTransform2f> odometry_to_map;
	return computeFootprintToObstacleDistance(target_pose, target_pose_in_merged_map, merged_map, distance_transformed_map, odometry_to_map);
}

bool firstTime = true;

float ScitosDrive::computeFootprintToObstacleDistance(const mira::Pose2& target_pose, mira::Pose2& target_pose_in_merged_map,
		mira::maps::OccupancyGrid& merged_map, mira::maps::GridMap<float>& distance_transformed_map, boost::shared_ptr<mira::RigidTransform2f>& odometry_to_map,
		bool debug_texts)
{
	if (firstTime) firstTime = false;
	debug_texts = true;
	// compute distance between footprint and closest obstacle

	// convert target_pose into coordinate system of merged map: for local merged_map, the coordinate transform is necessary, otherwise skip
	// todo: read out frames names for multi level buildings
	if (!odometry_to_map)
	{
		odometry_to_map = boost::make_shared<mira::RigidTransform2f>(robot_->getMiraAuthority().getTransform<mira::RigidTransform2f>("/maps/MapFrame", "/robot/OdometryFrame"));
	}

	// receive the merged map and merge with map
	const mira::RigidTransform2f map_to_odometry = odometry_to_map->inverse();
	if (merged_map.isEmpty())
	{
		mira::maps::OccupancyGrid merged_map_local = merged_map_channel_.read(/*mira::Time::now(), mira::Duration::seconds(1)*/)->value().clone(); // todo: enforce current data? need exception handling!
		if (debug_texts) std::cout << "merged map: " << merged_map.size() << "  "<<  merged_map.getCellSize() << " " <<  merged_map.getOffset() << std::endl;

		// add current local dynamic obstacles to global static map
		merged_map = map_.clone();
if (debug_texts) std::cout << "merged map cloned: " << merged_map.size() << "  "<<  merged_map.getCellSize() << " " <<  merged_map.getOffset() << std::endl;

		cv::threshold(merged_map, merged_map, 99, 255, CV_THRESH_BINARY);
		for (int v=0; v<merged_map_local.height(); ++v)
		{
			for (int u=0; u<merged_map_local.width(); ++u)
			{
				const mira::Point2f pm_m = merged_map_local.map2world(mira::Point2f(u,v));
				const mira::Point2f pw_m = map_to_odometry * pm_m;
				const mira::Point2i pw_px = merged_map.world2map(pw_m);
				merged_map(pw_px) = std::max(merged_map(pw_px), merged_map_local(u,v));
			}
		}
	}

	// create distance transform on merged map
	if (distance_transformed_map.isEmpty())
	{
		distance_transformed_map = mira::maps::GridMap<float>(merged_map.size(), merged_map.getCellSize(), merged_map.getOffset());
		if (debug_texts) std::cout << "distance_transformed_map: " << distance_transformed_map.size() << std::endl;
		cv::Mat threshold_img;
		//std::ofstream myfile;

		//myfile.open("/tmp/mat");
		//if (!firstTime)
		//std::cout << cv::format(merged_map.getMat(), cv::Formatter::FMT_CSV) << std::endl;
		//myfile.close();

		cv::threshold(merged_map, threshold_img, 130, 255, CV_THRESH_BINARY_INV);
		std::cout << "out threshold" << std::endl;
		cv::distanceTransform(threshold_img, distance_transformed_map, CV_DIST_L2, 5);
		if (debug_texts) std::cout << "distanceTransform done" << std::endl;
	}

	odometry_to_map = boost::shared_ptr<mira::RigidTransform2f>(new mira::RigidTransform2f(0, 0, 0));	// todo: hack: when working on a global map there is no transform necessary
	target_pose_in_merged_map = *odometry_to_map * target_pose;
	if (debug_texts) std::cout << "odometry_to_map : " << *odometry_to_map << std::endl;
	if (debug_texts) std::cout << "pose : " << target_pose_in_merged_map << std::endl;

	// computed closest obstacle distance
	mira::Point2i target_position_pixel = merged_map.world2map(target_pose_in_merged_map.t);
	const float obstacle_distance = collision_test_.distanceToObstacle(distance_transformed_map, target_position_pixel, target_pose_in_merged_map.phi()) * merged_map.getCellSize(); // in [m]
	if (debug_texts) std::cout << "target_position_pixel: (" << target_position_pixel.x() << ", " << target_position_pixel.y() << "),   distance: " << obstacle_distance << std::endl;

	return obstacle_distance;
}

void ScitosDrive::stopRobotAtCurrentPosition()
{
	mira::navigation::TaskPtr stopTask(new mira::navigation::Task());
	mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");

	// Task with big tolerance (360deg and 2 meters);
	const float position_tolerance = 0.4f; // todo (rmb-ma) get the size of the robot / link with the speed
	stopTask->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(robot_pose.x(), robot_pose.y()), position_tolerance, position_tolerance)));
	stopTask->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(robot_pose.yaw(), 2*PI)));
	const float wrong_direction_weight = 0.5f;
	stopTask->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, wrong_direction_weight)));
	robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", stopTask);
}

mira::Pose3 ScitosDrive::getRobotPose() const
{
	// todo: instead of "/maps/MapFrame" use the frame of the PositionTask (provide PositionTask to this function) for multi level buildings with multiple maps
	return robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
}

double ScitosDrive::computeEuclideanDistanceToGoal(const mira::Pose3& pose_a, const mira::Pose3& pose_b) const
{
	const float dx = pose_a.x() - pose_b.x();
	const float dy = pose_a.y() - pose_b.y();
	return sqrt(dx*dx + dy*dy);
}

bool ScitosDrive::isActionServerPreemptRequested(ScitosDrive::ActionServerType action_server_type) const {
	switch (action_server_type)
	{
		case ScitosDrive::WALL_FOLLOW_ACTION:
			return wall_follow_action_server_->isPreemptRequested();
		case ScitosDrive::MOVE_BASE_ACTION:
			return move_base_action_server_->isPreemptRequested();
		case ScitosDrive::PATH_ACTION:
			return path_action_server_->isPreemptRequested();
	}
	return false;
}

TargetCode ScitosDrive::waitForTargetApproach(const mira::Pose3& target_pose, float goal_position_tolerance, float goal_angle_tolerance,
		ScitosDrive::ActionServerType action_server_type, float cost_map_threshold)
{
	const double freeze_offset = 2.f; // [s]
	const double freeze_min_speed = 0.2f;
	double robot_freeze_timeout = freeze_offset + computeEuclideanDistanceToGoal(target_pose, getRobotPose());

	ros::Time last_robot_movement = ros::Time::now();
	while (true)
	{
		if (isActionServerPreemptRequested(action_server_type))
		{
			stopRobotAtCurrentPosition();
			return TARGET_GOAL_REACHED;
		}

		bool target_inaccessible = computeFootprintToObstacleDistance(mira::transform_cast<mira::Pose2>(target_pose)) <= 0.f;

		const mira::Pose3 robot_pose = getRobotPose();
		const double distance_to_goal = computeEuclideanDistanceToGoal(target_pose, robot_pose);
		const double delta_angle = normalize_angle(target_pose.yaw() - robot_pose.yaw());

		// also abort if the robot does not move for a long time
		double robot_speed_x = 0.; // [m/s]
		double robot_speed_theta = 0.;	// [rad/s]
		getCurrentRobotSpeed(robot_speed_x, robot_speed_theta);

		if (fabs(robot_speed_x) > 0.02 || fabs(robot_speed_theta) > 0.04)
			last_robot_movement = ros::Time::now();

		double robot_freeze_time = (ros::Time::now() - last_robot_movement).toSec();

		boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);

		if (distance_to_goal < goal_position_tolerance && fabs(delta_angle) < goal_angle_tolerance)
			return TARGET_GOAL_REACHED;
		if (target_inaccessible)
			return TARGET_INACESSIBLE;
		if (nav_pilot_event_status_.compare("PlanAndDrive") != 0)
			return TARGET_PILOT_NOT_IN_PLAN_AND_DRIVE_MODE;
		if (robot_freeze_time > robot_freeze_timeout)
			return TARGET_ROBOT_DOES_NOT_MOVE;

		ros::spinOnce();
	}
	return TARGET_GOAL_REACHED;
}

void ScitosDrive::publishComputedTarget(const tf::StampedTransform& transform)
{
	geometry_msgs::TransformStamped transform_msg;
	tf::transformStampedTFToMsg(transform, transform_msg);
	computed_trajectory_pub_.publish(transform_msg);
}

void ScitosDrive::publishCommandedTarget(const tf::StampedTransform& transform)
{
	geometry_msgs::TransformStamped transform_msg;
	tf::transformStampedTFToMsg(transform, transform_msg);
	commanded_trajectory_pub_.publish(transform_msg);
}
#endif

mira::Pose2 ScitosDrive::computeLeftCandidate(const mira::Pose2 &target_pose_in_merged_map, const double offset, cv::Point2d direction_left,
		const mira::maps::OccupancyGrid &merged_map,
		const mira::RigidTransform2f &map_to_odometry, const cv::Point2d &map_world_offset_, double map_resolution_,
		const double min_obstacle_distance, const cv::Mat &area_map,
		const mira::maps::GridMap<float> &distance_transformed_map, bool &found) const
{
	mira::Pose2 pose_left_in_merged_map(target_pose_in_merged_map.x()+offset*direction_left.x, target_pose_in_merged_map.y()+offset*direction_left.y, target_pose_in_merged_map.phi());
	mira::Point2i target_position_pixel = merged_map.world2map(pose_left_in_merged_map.t);
	float obstacle_distance = collision_test_.distanceToObstacle(distance_transformed_map, target_position_pixel, target_pose_in_merged_map.phi()) * merged_map.getCellSize(); // in [m]
	mira::Pose2 target_pose_candidate = map_to_odometry * pose_left_in_merged_map;
	cv::Point map_coordinates = cv::Point((target_pose_candidate.x()+map_world_offset_.x)/map_resolution_, (target_pose_candidate.y()+map_world_offset_.y)/map_resolution_);

	found = obstacle_distance >= min_obstacle_distance && area_map.at<uchar>(map_coordinates)==255;
	return target_pose_candidate;
}

mira::Pose2 ScitosDrive::computeRightCandidate(const mira::Pose2 &target_pose_in_merged_map, const double offset, cv::Point2d direction_left,
		const mira::maps::OccupancyGrid &merged_map,
		const mira::RigidTransform2f &map_to_odometry, const cv::Point2d &map_world_offset_, double map_resolution_,
		const double min_obstacle_distance, const cv::Mat &area_map,
		const mira::maps::GridMap<float> &distance_transformed_map, bool &found) const
{
	return computeLeftCandidate(target_pose_in_merged_map, -offset, direction_left, merged_map, map_to_odometry, map_world_offset_, map_resolution_, min_obstacle_distance,
			area_map, distance_transformed_map, found);
}

bool ScitosDrive::computeAlternativeTargetIfNeeded(mira::Pose3 &target_pose, const double next_x, const double next_y, const double min_obstacle_distance, const cv::Mat& area_map)
{
	mira::Pose2 target_pose_in_merged_map;
	mira::maps::OccupancyGrid merged_map;
	mira::maps::GridMap<float> distance_transformed_map;
	boost::shared_ptr<mira::RigidTransform2f> odometry_to_map;
	float obstacle_distance = computeFootprintToObstacleDistance(mira::transform_cast<mira::Pose2>(target_pose), target_pose_in_merged_map, merged_map, distance_transformed_map, odometry_to_map);

	if (obstacle_distance >= min_obstacle_distance) return true;

	std::cout << "  -  Target (" << target_pose.x() << ", " << target_pose.y() << "),   distance: " << obstacle_distance << "    is not accessible." << std::endl;
	// try to shift target perpendicular to driving direction,
	// accept the closest accessible target left or right which is still located within the allowed area of path->area_map
	const cv::Point2d direction_left(-sin(target_pose_in_merged_map.phi()), cos(target_pose_in_merged_map.phi()));
	mira::RigidTransform2f map_to_odometry = odometry_to_map->inverse();
	bool alternative_target_found = false;

	const double max_track_offset = 2.0 * 0.9;		// in [m]	// todo: param
	for (double offset = merged_map.getCellSize(); offset <= max_track_offset; offset += merged_map.getCellSize())
	{
		const double x = target_pose.x();
		const double y = target_pose.y();

		bool found_left;
		mira::Pose2 target_left_candidate = computeLeftCandidate(target_pose_in_merged_map, offset, direction_left, merged_map, map_to_odometry, map_world_offset_, map_resolution_,
							min_obstacle_distance, area_map, distance_transformed_map, found_left);

		bool found_right;
		mira::Pose2 target_right_candidate = computeRightCandidate(target_pose_in_merged_map, offset, direction_left, merged_map, map_to_odometry, map_world_offset_, map_resolution_,
							min_obstacle_distance, area_map, distance_transformed_map, found_right);

		if (!found_left && !found_right) continue;

		// use of determinant between vectors target_pose and next_pose
		if (x*next_y - y*next_x < 0) // left
			target_pose = mira::transform_cast<mira::Pose3>(found_left ? target_left_candidate : target_right_candidate);
		else
			target_pose = mira::transform_cast<mira::Pose3>(found_right ? target_right_candidate : target_left_candidate);

		return true;
	}
	return false;
}

// Option with normal move commands
void ScitosDrive::path_callback(const scitos_msgs::MoveBasePathGoalConstPtr& path)
{
#ifdef __WITH_PILOT__
	/*
	 * https://www.mira-project.org/MIRA-doc/toolboxes/Navigation/classmira_1_1navigation_1_1PathFollowTask.html#_details
	 * http://www.mira-project.org/MIRA-doc/domains/tutorials/WaypointVisitor/index.html
	 */
	ROS_INFO("ScitosDrive::path_callback: Following a path.");

	const double desired_planning_ahead_time = 0.4;	// used for computing goal_position_tolerance, in [s]

	float goal_position_tolerance = (path->goal_position_tolerance > 0.f ? path->goal_position_tolerance : 0.1f);
	const float goal_angle_tolerance = (path->goal_angle_tolerance > 0.f ? path->goal_angle_tolerance : 0.17f);

	// convert the area_map msg in cv format
	cv::Mat area_map; map_msgToCvFormat(path->area_map, area_map);

	for (size_t i=0; i<path->target_poses.size(); ++i)
	{
		mira::Time start_time = mira::Time::now();

		if (path_action_server_->isPreemptRequested())
		{
			PathActionServer::Result res;
			res.last_planned_point_index = i;
			path_action_server_->setSucceeded(res);
			return;
		}

		const geometry_msgs::Pose pose_ros = path->target_poses[i].pose;

		// convert target pose to mira::Pose3
		mira::Pose3 target_pose3(Eigen::Vector3f(pose_ros.position.x, pose_ros.position.y, pose_ros.position.z),
				Eigen::Quaternionf(pose_ros.orientation.w, pose_ros.orientation.x, pose_ros.orientation.y, pose_ros.orientation.z));

		// publish computed next target
		publishComputedTarget(tf::StampedTransform(tf::Transform(tf::Quaternion(pose_ros.orientation.x, pose_ros.orientation.y, pose_ros.orientation.z, pose_ros.orientation.w),
				tf::Vector3(pose_ros.position.x, pose_ros.position.y, pose_ros.position.z)), ros::Time::now(), map_frame_, robot_frame_));

		const double cost_map_threshold = -1.0;	// deactivates the cost map
		const double min_obstacle_distance = 0.02;	// in [m]
		// in order to see if the robot will turn on the left or on the right
		const double next_x = i < path->target_poses.size() - 1 ? path->target_poses[i+1].pose.position.x : 0;
		const double next_y = i < path->target_poses.size() - 1 ? path->target_poses[i+1].pose.position.y : 0;
		bool found = computeAlternativeTargetIfNeeded(target_pose3, next_x, next_y, min_obstacle_distance, area_map);

		if (!found)
		{
			std::cout << "  -> No alternative target found." << std::endl;
			continue;
		}

		// publish commanded next target
		publishCommandedTarget(tf::StampedTransform(tf::Transform(tf::Quaternion(target_pose3.r.x(), target_pose3.r.y(), target_pose3.r.z(), target_pose3.r.w()), tf::Vector3(target_pose3.x(), target_pose3.y(), target_pose3.z())), ros::Time::now(), map_frame_, robot_frame_));

		// get current robot speed
		double robot_speed_x = 0.; // [m/s]
		double robot_speed_theta = 0.;	// [rad/s]
		getCurrentRobotSpeed(robot_speed_x, robot_speed_theta);

		// adapt position task accuracy to robot speed -> the faster the robot moves the more accuracy is needed
		const double goal_accuracy = 0.05 + 0.2 * std::max(0., max_speed_x-fabs(robot_speed_x))/max_speed_x;	// only takes effect if the robot ever reaches the goal exactly instead of being commanded to the next goal already
		// todo: if there is a big distance between two successive goal positions, decrease the goal tolerance
		goal_position_tolerance = 0.4 + robot_speed_x * desired_planning_ahead_time;

		const double angle_accuracy = 5*PI/180;
		setTaskAndWaitForTarget(target_pose3, goal_accuracy, goal_position_tolerance, angle_accuracy, goal_angle_tolerance, ScitosDrive::PATH_ACTION, cost_map_threshold);

	}

	std::cout << "  Path following successfully terminated." << std::endl;

	// this sends the response back to the caller
	PathActionServer::Result res;

	res.last_planned_point_index = path->target_poses.size();
	path_action_server_->setSucceeded(res);
#else
	ROS_ERROR("ScitosDrive::path_callback: This function is not compiled. Install the MIRA Pilot addon and make sure it is found by cmake.");

	// this sends the response back to the caller
	PathActionServer::Result res;
	path_action_server_->setAborted(res);
#endif
}

void ScitosDrive::map_msgToCvFormat(const sensor_msgs::Image& image_map, cv::Mat& map) const
{
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(image_map, sensor_msgs::image_encodings::MONO8);
	map = cv_ptr_obj->image;
}

// todo rmb-ma (check the wrong orientation bug and how to solve it in this function)
bool ScitosDrive::computeClosestPos(const cv::Mat& level_set_map, const cv::Mat& driving_direction, const cv::Point& current_pos, cv::Point& best_pos) const
{
	double min_dist_sqr = 1e10;
	for (int v = 0; v < level_set_map.rows; ++v)
	{
		for (int u = 0; u < level_set_map.cols; ++u)
		{
			if (level_set_map.at<uchar>(v, u) != 255) continue;

			const double dx = u - current_pos.x;
			const double dy = v - current_pos.y;
			const double dist_sqr = dx*dx + dy*dy;

			if (dist_sqr >= min_dist_sqr) continue;
			const double dd_x = cos(driving_direction.at<float>(v, u));
			const double dd_y = sin(driving_direction.at<float>(v, u));

			const int range = 10;
			bool found_pixels = false;
			for (int du = -range; du <= range; ++du) {
				for (int dv = -range; dv <= range; ++dv) {
					if (du == 0 && dv == 0) continue;
					const int uu = u + du;
					const int vv = v + dv;

					if ((du*dd_y + dv*dd_x)/sqrt(du*du + dv*dv) > -0.3) continue;
					if (level_set_map.at<uchar>(vv, uu) != 255) continue;
					found_pixels = true;
					break;
				}
			}

			if (found_pixels) continue;

			//std::cout << "(" << u << ", " << v << ", 1.)" << std::endl;
			min_dist_sqr = dist_sqr;
			best_pos = cv::Point(u, v);
		}
	}
	if (min_dist_sqr != 1e10)
		return true;

	for (int v = 0; v < level_set_map.rows; ++v)
	{
		for (int u = 0; u < level_set_map.cols; ++u)
		{
			if (level_set_map.at<uchar>(v, u) != 255) continue;

			const double dx = u - current_pos.x;
			const double dy = v - current_pos.y;
			const double dist_sqr = dx*dx + dy*dy;

			if (dist_sqr >= min_dist_sqr) continue;

			min_dist_sqr = dist_sqr;
			best_pos = cv::Point(u, v);
		}
	}

	return min_dist_sqr != 1e10;
}
bool ScitosDrive::computePosInNeighborhoodWithMaxCosinus(cv::Mat& level_set_map, const cv::Point& current_pos, cv::Point& next_pos, const cv::Mat& driving_direction) const
{
	double max_cos_angle = -1e10;
	const double dd_x = cos(driving_direction.at<float>(current_pos));
	const double dd_y = sin(driving_direction.at<float>(current_pos));
	for (int dv = -1; dv <= 1; ++dv)
	{
		for (int du = -1; du <= 1; ++du)
		{
			const int nu = current_pos.x + du;
			const int nv = current_pos.y + dv;

			// the last condition implicitly excludes the center pixel
			if (nu < 0 || nu >= level_set_map.cols || nv < 0 || nv > level_set_map.rows || level_set_map.at<uchar>(nv,nu) != 255)
				continue;

			level_set_map.at<uchar>(nv, nu) = 0;		// mark all neighboring points as visited

			// determine the angle difference
			double cos_angle = dd_x*du + dd_y*dv;
			if (cos_angle < 0) continue;

			if (cos_angle > max_cos_angle)
			{
				max_cos_angle = cos_angle;
				next_pos = cv::Point(nu, nv);
			}
		}
	}
	return max_cos_angle != -1e10;
}

cv::Vec3d mapPosToWallGoal(const cv::Mat& driving_direction, const cv::Point& map_pos, double map_resolution, const cv::Point& map_origin)
{
	return cv::Vec3d(map_pos.x*map_resolution + map_origin.x, map_pos.y*map_resolution + map_origin.y, driving_direction.at<float>(map_pos));
}
void ScitosDrive::displayWallFollowerPath(const std::vector<cv::Vec3d>& wall_poses, const cv::Mat& area_map, double map_resolution, const cv::Point& map_origin) const

{
	for(size_t step = wall_poses.size() - 1; step < wall_poses.size(); ++step)
	{
		cv::Mat fov_path_map = area_map.clone();
		cv::resize(fov_path_map, fov_path_map, cv::Size(), 2, 2, cv::INTER_LINEAR);
		cv::circle(fov_path_map, 2*cv::Point((wall_poses[0].val[0] - map_origin.x)/map_resolution, (wall_poses[0].val[1]-map_origin.y)/map_resolution), 2, cv::Scalar(0.6), CV_FILLED);
		for(size_t i = 1; i <= step; ++i)
		{
			cv::Point p1((wall_poses[i-1].val[0]-map_origin.x)/map_resolution, (wall_poses[i-1].val[1]-map_origin.y)/map_resolution);
			cv::Point p2((wall_poses[i].val[0]-map_origin.x)/map_resolution, (wall_poses[i].val[1]-map_origin.y)/map_resolution);
			cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(0.8), CV_FILLED);
			cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(0.6), 1);
			if (i == step)
			{
				cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(0.3), CV_FILLED);
				cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(0.6), 1);
				cv::Point p3(p2.x+5*cos(wall_poses[i].val[2]), p2.y+5*sin(wall_poses[i].val[2]));
				cv::line(fov_path_map, 2*p2, 2*p3, cv::Scalar(0.2), 1);
			}
		}
		if (step == wall_poses.size()-1)
		{
			//cv::imshow("cell path", fov_path_map);
			//cv::waitKey(20);
		}
	}
}

void ScitosDrive::computeWallPosesDense(const scitos_msgs::MoveBaseWallFollowGoalConstPtr& goal, std::vector<cv::Vec3d>& wall_poses_dense) const
{
	const double map_resolution = goal->map_resolution;	// in [m/cell]
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
	const double target_wall_distance_px = (goal->target_wall_distance ? 0.1 + robot_radius_ +  goal->target_wall_distance : 0.55) / map_resolution;

	// allowed deviation from target distance of robot center to wall, used for sampling goal poses along the wall
	const double target_wall_distance_px_epsilon = 1;

	cv::Mat map; map_msgToCvFormat(goal->map, map);
	cv::Mat area_map; map_msgToCvFormat(goal->area_map, area_map);
	cv::Mat coverage_map; map_msgToCvFormat(goal->coverage_map, coverage_map);

	// distance-transformed map, type: CV_32FC1
	cv::Mat distance_map; cv::distanceTransform(area_map, distance_map, CV_DIST_L2, 5);
	cv::Mat distance_map_disp; cv::convertScaleAbs(distance_map, distance_map_disp);	// conversion to 8 bit image

	// find the points near walls in the accessible area of the room
	cv::Mat level_set_map = cv::Mat::zeros(distance_map.rows, distance_map.cols, CV_8UC1);

	// determine a preferred driving direction for each point
	// todo: param: the direction offset is -pi/2 for wall following on the right side and +pi/2 for wall following on the left side
	const double direction_offset = -0.5*PI;
	cv::Mat distance_map_dx, distance_map_dy;
	cv::Sobel(distance_map_disp, distance_map_dx, CV_64F, 1, 0, 3);
	cv::Sobel(distance_map_disp, distance_map_dy, CV_64F, 0, 1, 3);
	cv::Mat driving_direction(distance_map.rows, distance_map.cols, CV_32FC1);

	for (int v = 0; v < distance_map.rows; ++v)
	{
		for (int u = 0; u < distance_map.cols; ++u)
		{
			if (coverage_map.at<uchar>(v, u) == 255) continue;
			if (fabs(distance_map.at<float>(v,u) - target_wall_distance_px) >= target_wall_distance_px_epsilon) continue;

			level_set_map.at<uchar>(v,u) = 255;
			driving_direction.at<float>(v,u) = normalize_angle(atan2(distance_map_dy.at<double>(v,u), distance_map_dx.at<double>(v,u)) + direction_offset);
		}
	}

	mira::Pose3 robot_pos = getRobotPose();
	cv::Point2d robot_pos_in_map((robot_pos.x() - map_origin.x)/map_resolution, (robot_pos.y() - map_origin.y) / map_resolution);

	cv::Point current_pos;
	if (!computeClosestPos(level_set_map, driving_direction, robot_pos_in_map, current_pos))
		return;

	wall_poses_dense.push_back(mapPosToWallGoal(driving_direction, current_pos, map_resolution, map_origin));
	level_set_map.at<uchar>(current_pos) = 0;

	// used to mask all cells in the neighborhood of already visited cells
	// write driving direction into visited_map and only skip if driving direction is similar
	cv::Mat visited_map = -1e11*cv::Mat::ones(level_set_map.rows, level_set_map.cols, CV_32FC1);
	while (true)
	{
		cv::Point next_pos(-1,-1);
		if (!computePosInNeighborhoodWithMaxCosinus(level_set_map, current_pos, next_pos, driving_direction))
		{
			if (!computeClosestPos(level_set_map, driving_direction, current_pos, next_pos))
				break;
		}

		level_set_map.at<uchar>(next_pos) = 0;
		// do not visit places a second time
		// except with very different driving direction
		if (visited_map.at<float>(next_pos) < -1e10 || fabs(normalize_angle(driving_direction.at<float>(next_pos)-visited_map.at<float>(next_pos))) > 100./180.*PI)
		{
			wall_poses_dense.push_back(cv::Vec3d(next_pos.x*map_resolution + map_origin.x, next_pos.y*map_resolution + map_origin.y, driving_direction.at<float>(next_pos)));
			cv::circle(visited_map, next_pos, 3, cv::Scalar(driving_direction.at<float>(next_pos)), -1);
		}
		current_pos = next_pos;
	}
}

// Wall following function
void ScitosDrive::wall_follow_callback(const scitos_msgs::MoveBaseWallFollowGoalConstPtr& goal)
{
#ifdef __WITH_PILOT__
	ROS_INFO("ScitosDrive::wall_follow_callback: Driving along a wall.");

	// collect the wall following path
	std::vector<cv::Vec3d> wall_poses_dense;
	computeWallPosesDense(goal, wall_poses_dense);

	if (wall_poses_dense.empty())
		return;

	std::vector<cv::Vec3d> wall_poses;
	// reduce density of wall_poses
	wall_poses.push_back(wall_poses_dense[0]);
	size_t last_used_pose_index = 0;
	for (size_t i = 1; i < wall_poses_dense.size(); ++i)
	{
		const cv::Vec3d& previous = wall_poses_dense[last_used_pose_index];
		const cv::Vec3d& next = wall_poses_dense[i];
		const double dx = next.val[0] - previous.val[0];
		const double dy = next.val[1] - previous.val[1];

		if (dx*dx + dy*dy > 0.16*0.16 || normalize_angle(next.val[2] - previous.val[2]) > 1.5708)
		{
			wall_poses.push_back(wall_poses_dense[i]);
			last_used_pose_index = i;
		}
	}

	if (false)
	{
		const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
		cv::Mat area_map; map_msgToCvFormat(goal->area_map, area_map);
		displayWallFollowerPath(wall_poses, area_map, goal->map_resolution, map_origin);
	}

	// move through the wall poses
	const float path_tolerance = goal->path_tolerance > 0.f ? goal->path_tolerance : 0.1f;
	const float goal_position_tolerance = goal->goal_position_tolerance > 0.f ? goal->goal_position_tolerance : 0.1f;
	const float goal_angle_tolerance = goal->goal_angle_tolerance > 0.f ? goal->goal_angle_tolerance : 0.17f;
	// target distance between robot and wall during wall following, in [m]
	const float target_wall_distance = goal->target_wall_distance >= 0.f ? goal->target_wall_distance : 0.1f;
	// when traveling farther than this threshold distance, the robot does not use the wall following objective, in [m]
	const float wall_following_off_traveling_distance_threshold = goal->wall_following_off_traveling_distance_threshold >= 0.f ?
			goal->wall_following_off_traveling_distance_threshold :
			1.0f;
	const double cost_map_threshold = -1.;

	for (unsigned k = 0; k < wall_poses.size(); ++k)
	{
		if (wall_follow_action_server_->isPreemptRequested())
		{
			WallFollowActionServer::Result res;
			wall_follow_action_server_->setAborted(res);
			return;
		}

		mira::Pose3 target_pose3(wall_poses[k].val[0], wall_poses[k].val[1], 0., wall_poses[k].val[2], 0., 0.);

		publishComputedTarget(tf::StampedTransform(tf::Transform(
				tf::Quaternion(target_pose3.r.x(), target_pose3.r.y(), target_pose3.r.z(), target_pose3.r.w()),
				tf::Vector3(target_pose3.x(), target_pose3.y(), target_pose3.z())), ros::Time::now(), map_frame_, robot_frame_));

		const mira::Pose3 robot_pos = getRobotPose();
		const double dx = robot_pos.x() - target_pose3.x();
		const double dy = robot_pos.y() - target_pose3.y();

		const double angle_robot_to_pose = dx*cos(target_pose3.yaw()) + dy*sin(target_pose3.yaw());

		publishCommandedTarget(tf::StampedTransform(tf::Transform(
				tf::Quaternion(target_pose3.r.x(), target_pose3.r.y(), target_pose3.r.z(), target_pose3.r.w()),
				tf::Vector3(target_pose3.x(), target_pose3.y(), target_pose3.z())), ros::Time::now(), map_frame_, robot_frame_));

		double robot_speed_x = 0.; // [m/s]
		double robot_speed_theta = 0.;	// [rad/s]
		getCurrentRobotSpeed(robot_speed_x, robot_speed_theta);

		// adapt position task accuracy to robot speed -> the faster the robot moves the more accuracy is needed
		const double max_speed = 0.3;
		const double goal_accuracy = 0.05 + 0.2 * std::max(0., max_speed - fabs(robot_speed_x))/max_speed;
		const double angle_accuracy = PI / 2;

		const mira::Pose3 robot_pose = getRobotPose();
		const double cos_angle = cos(wall_poses[k].val[2] - robot_pose.yaw());
		const double current_target_wall_distance = cos_angle > 0.4 && dx*dx + dy*dy < 0.7*0.7 ? target_wall_distance : -1;
		std::cout << "current_target_wall_distance " << current_target_wall_distance << std::endl;

		setTaskAndWaitForTarget(target_pose3, goal_accuracy, goal_position_tolerance, angle_accuracy, goal_angle_tolerance,
				ScitosDrive::WALL_FOLLOW_ACTION, cost_map_threshold, current_target_wall_distance);
	}

	std::cout << "  Wall following successfully terminated." << std::endl;

	// this sends the response back to the caller
	WallFollowActionServer::Result res;
	wall_follow_action_server_->setSucceeded(res);

#else
	ROS_ERROR("ScitosDrive::wall_follow_callback: This function is not compiled. Install the MIRA Pilot addon and make sure it is found by cmake.");

	// this sends the response back to the caller
	WallFollowActionServer::Result res;
	wall_follow_action_server_->setAborted(res);
#endif
}

bool ScitosDrive::reset_motor_stop(scitos_msgs::ResetMotorStop::Request  &req, scitos_msgs::ResetMotorStop::Response &res) {
  //  call_mira_service
  emergency_stop_.data = false;
  emergency_stop_pub_.publish(emergency_stop_);
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("resetMotorStop"));
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  return true;
}

bool ScitosDrive::reset_odometry(scitos_msgs::ResetOdometry::Request  &req, scitos_msgs::ResetOdometry::Response &res) {
  //  call_mira_service
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("resetOdometry"));
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  return true;
}

bool ScitosDrive::emergency_stop(scitos_msgs::EmergencyStop::Request  &req, scitos_msgs::EmergencyStop::Response &res) {
  //  call_mira_service
  emergency_stop_.data = true;
  emergency_stop_pub_.publish(emergency_stop_);
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("emergencyStop"));
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  return true;
}

bool ScitosDrive::enable_motors(scitos_msgs::EnableMotors::Request  &req, scitos_msgs::EnableMotors::Response &res) {
  //  call_mira_service
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("enableMotors"),(bool)req.enable);
  r.timedWait(mira::Duration::seconds(1));
  r.get();

  return true;
}

bool ScitosDrive::change_force(scitos_msgs::ChangeForce::Request  &req, scitos_msgs::ChangeForce::Response &res) {
	// change mira params
	return set_mira_param_("MainControlUnit.Force",mira::toString(req.force));
}

bool ScitosDrive::enable_rfid(scitos_msgs::EnableRfid::Request  &req, scitos_msgs::EnableRfid::Response &res) {
  return set_mira_param_("MainControlUnit.RearLaser.Enabled", req.enable ? "true" : "false");
}

bool ScitosDrive::reset_barrier_stop(scitos_msgs::ResetBarrierStop::Request  &req, scitos_msgs::ResetBarrierStop::Response &res) {
  barrier_status_.barrier_stopped = false;
  return true;
}
