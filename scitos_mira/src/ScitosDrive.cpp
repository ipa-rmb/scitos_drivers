
#include "scitos_mira/ScitosDrive.h"
#include "scitos_mira/ScitosG5.h"

#include <transform/RigidTransform.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt64.h>

#include <opencv2/opencv.hpp>

//#define DISABLE_MOVEMENTS	// todo: remove

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

  robot_->getMiraAuthority().subscribe<mira::robot::Odometry2>("/robot/Odometry", //&ScitosBase::odometry_cb);
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
  map_frame_ = "map_original";
  map_clean_frame_ = "map";
  map_segmented_frame_ = "map_segmented";
  robot_frame_ = "base_link";
  map_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>(map_frame_, 1, true);
  map_clean_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>(map_clean_frame_, 1, true);
  map_segmented_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>(map_segmented_frame_, 1, true);
  robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/static/Map", &ScitosDrive::map_data_callback, this);
  robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/cleaning/Map", &ScitosDrive::map_clean_data_callback, this);	// todo: hack:
  robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/segmentation/Map", &ScitosDrive::map_segmented_data_callback, this);
  robot_->getMiraAuthority().subscribe<mira::maps::GridMap<double,1> >("/maps/cost/PlannerMap_FinalCostMap", &ScitosDrive::cost_map_data_callback, this);
  computed_trajectory_pub_ = robot_->getRosNode().advertise<geometry_msgs::TransformStamped>("/room_exploration/coverage_monitor_server/computed_target_trajectory_monitor", 1);
  commanded_trajectory_pub_ = robot_->getRosNode().advertise<geometry_msgs::TransformStamped>("/room_exploration/coverage_monitor_server/commanded_target_trajectory_monitor", 1);

  // read in robot radius, coverage radius, and coverage area center offset against robot base link
  writeParametersToROSParamServer();
#endif
#ifndef DISABLE_MOVEMENTS
  cmd_vel_subscriber_ = robot_->getRosNode().subscribe("/cmd_vel", 1000, &ScitosDrive::velocity_command_callback, this);
#endif
  // mira navigation data types: http://www.mira-project.org/MIRA-doc/domains/robot/SCITOSConfigs/index.html
  move_base_action_server_ = boost::shared_ptr<MoveBaseActionServer>(new MoveBaseActionServer(robot_->getRosNode(), "/move_base", boost::bind(&ScitosDrive::move_base_callback, this, _1), false)); // this initializes the action server; important: always set the last parameter to false
  move_base_action_server_->start();

  path_action_server_ = boost::shared_ptr<PathActionServer>(new PathActionServer(robot_->getRosNode(), "/move_base_path", boost::bind(&ScitosDrive::path_callback, this, _1), false)); // this initializes the action server; important: always set the last parameter to false
  path_action_server_->start();

  wall_follow_action_server_ = boost::shared_ptr<PathActionServer>(new PathActionServer(robot_->getRosNode(), "/move_base_wall_follow", boost::bind(&ScitosDrive::wall_follow_callback, this, _1), false)); // this initializes the action server; important: always set the last parameter to false
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
}

void ScitosDrive::velocity_command_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  if ( !barrier_status_.barrier_stopped && !emergency_stop_.data) {
      mira::RigidTransform<float, 2> speed(msg->linear.x, 0, msg->angular.z);
#ifndef DISABLE_MOVEMENTS
      mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", "setVelocity", speed);
      r.wait();
#endif
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
	mira::RPCFuture<mira::robot::RobotModelPtr> r = robot_->getMiraAuthority().callService<mira::robot::RobotModelPtr>("/robot/Robot", "getRobotModel");
	bool success = r.timedWait(mira::Duration::seconds(10));
	std::cout << "########## Robot Configuration ##########" << std::endl;
	robot_radius_ = 0.325;
	coverage_radius_ = 0.25;
	coverage_offset_ = mira::RigidTransform3d(0.29, -0.114, 0.0, 0.0, 0.0, 0.0);
	if (success==true)
	{
		mira::robot::RobotModelPtr robot_model = r.get();
		if (robot_model)
		{
			robot_radius_ = robot_model->getFootprint().getInnerRadius();
			mira::Footprint coverage_footprint = robot_model->getFootprint("", mira::Time::now(), "CleaningTool");
			coverage_radius_ = 0.5*(coverage_footprint.getOuterRadius()-coverage_footprint.getInnerRadius());	// todo: hack: only works for circle footprints
			coverage_offset_ = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/modules/brushcleaning/BrushFrame", "/robot/RobotFrame");
		}
		else
			std::cout << "Error: could not read robot parameters. Taking standard values." << std::endl;
	}
	else
		std::cout << "Error: could not read robot parameters. Taking standard values." << std::endl;
	std::cout << "robot_radius=" << robot_radius_ << "   coverage_radius=" << coverage_radius_ << "   coverage_offset=(" << coverage_offset_.x() << ", " << coverage_offset_.y() << ")\n" << std::endl;

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
	ROS_INFO_STREAM("Received map " << map_frame_);
	publish_grid_map(data->value(), map_pub_, map_frame_);
}

void ScitosDrive::map_clean_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data)
{
	// convert map to ROS format
	ROS_INFO_STREAM("Received map " << map_clean_frame_);
	publish_grid_map(data->value(), map_clean_pub_, map_clean_frame_);	// todo: hack: using a separately edited map as the "real" map for planning
}

void ScitosDrive::map_segmented_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data)
{
	// convert map to ROS format
	ROS_INFO_STREAM("Received map " << map_segmented_frame_);
	publish_grid_map(data->value(), map_segmented_pub_, map_segmented_frame_);
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
	int i=0;
	for (int v=0; v<map.rows; ++v)
	{
		unsigned char* map_ptr = map.ptr(v);
		for (int u=0; u<map.cols; ++u, ++i, ++map_ptr)
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
//	cv::imshow(frame_id, map);
//	cv::waitKey();
}

void ScitosDrive::cost_map_data_callback(mira::ChannelRead<mira::maps::GridMap<double,1> > data)
{
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

void ScitosDrive::move_base_callback(const move_base_msgs::MoveBaseGoalConstPtr& goal)
{
#ifdef __WITH_PILOT__
	Eigen::Affine3d goal_pose;
	tf::poseMsgToEigen(goal->target_pose.pose, goal_pose);
	Eigen::Vector3d euler_angles = goal_pose.rotation().eulerAngles(2,1,0);	// computes yaw, pitch, roll angles from rotation matrix

	mira::navigation::TaskPtr task(new mira::navigation::Task());
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(goal->target_pose.pose.position.x, goal->target_pose.pose.position.y), 0.1f, 0.1f)));
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(euler_angles(0), 0.1)));
	task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));

	// Set this as our goal. Will cause the robot to start driving.
	mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
	r.wait();

	// this sends the response back to the caller
	MoveBaseActionServer::Result res;
	move_base_action_server_->setSucceeded(res);
#else
	ROS_ERROR("ScitosDrive::move_base_callback: This function is not compiled. Install the MIRA Pilot addon and make sure it is found by cmake.");

	// this sends the response back to the caller
	MoveBaseActionServer::Result res;
	move_base_action_server_->setAborted(res);
#endif
}

double ScitosDrive::normalize_angle(double delta_angle)
{
	const double pi = 3.14159265359;
	while (delta_angle < -pi)
		delta_angle += 2*pi;
	while (delta_angle > pi)
		delta_angle -= 2*pi;
	return delta_angle;
}

#ifdef __WITH_PILOT__
void ScitosDrive::getCurrentRobotSpeed(double& robot_speed_x, double& robot_speed_theta)
{
	boost::mutex::scoped_lock lock(odom_msg_mutex_);
	robot_speed_x = odom_msg_.twist.twist.linear.x;
	robot_speed_theta = odom_msg_.twist.twist.angular.z;
}


void ScitosDrive::waitForTargetApproach(const mira::Pose3& target_pose, const float goal_position_tolerance, const float goal_angle_tolerance)
{
	double robot_freeze_timeout = -1.;	// [s]
	ros::Time last_robot_movement = ros::Time::now();
	while (true)
	{
		mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
		const double distance_to_goal = sqrt((target_pose.x()-robot_pose.x())*(target_pose.x()-robot_pose.x()) + (target_pose.y()-robot_pose.y())*(target_pose.y()-robot_pose.y()));
		const double delta_angle = normalize_angle(target_pose.yaw()-robot_pose.yaw());
		//std::cout << "      - current robot pose: " << robot_pose.t(0) << ", " << robot_pose.t(1) << ", " << robot_pose.yaw() << "    dist=" << distance_to_goal << "     phi=" << delta_angle << std::endl;

		// flexible timeout on movements based on distance to goal, separate timeout function
		if (robot_freeze_timeout < 0.)
			robot_freeze_timeout = 2. + 0.2*distance_to_goal;

		// also abort if the robot does not move for a long time
		double robot_speed_x = 0.; // [m/s]
		double robot_speed_theta = 0.;	// [rad/s]
		getCurrentRobotSpeed(robot_speed_x, robot_speed_theta);
		if (fabs(robot_speed_x) > 0.01 || fabs(robot_speed_theta) > 0.01)
			last_robot_movement = ros::Time::now();
		double robot_freeze_time = (ros::Time::now()-last_robot_movement).toSec();
		//std::cout << "robot_freeze_time" << robot_freeze_time << std::endl;
		boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);
		if (nav_pilot_event_status_.compare("PlanAndDrive")!=0 ||
				(distance_to_goal<goal_position_tolerance && fabs(delta_angle)<goal_angle_tolerance) ||
				(robot_freeze_time > robot_freeze_timeout))
			break;
		ros::spinOnce();
	}
}

void ScitosDrive::publishComputedTarget(const tf::StampedTransform& transform)
{
	// publish commanded next target
	geometry_msgs::TransformStamped transform_msg;
	tf::transformStampedTFToMsg(transform, transform_msg);
	computed_trajectory_pub_.publish(transform_msg);
}

void ScitosDrive::publishCommandedTarget(const tf::StampedTransform& transform)
{
	// publish commanded next target
	geometry_msgs::TransformStamped transform_msg;
	tf::transformStampedTFToMsg(transform, transform_msg);
	commanded_trajectory_pub_.publish(transform_msg);
}
#endif

// Option with normal move commands
void ScitosDrive::path_callback(const scitos_msgs::MoveBasePathGoalConstPtr& path)
{
#ifdef __WITH_PILOT__
	/*
	 * https://www.mira-project.org/MIRA-doc/toolboxes/Navigation/classmira_1_1navigation_1_1PathFollowTask.html#_details
	 * http://www.mira-project.org/MIRA-doc/domains/tutorials/WaypointVisitor/index.html
	 */
	ROS_INFO("ScitosDrive::path_callback: Following a path.");

	const float path_tolerance = (path->path_tolerance>0.f ? path->path_tolerance : 0.1f);
	const float goal_position_tolerance = (path->goal_position_tolerance>0.f ? path->goal_position_tolerance : 0.1f);
	const float goal_angle_tolerance = (path->goal_angle_tolerance>0.f ? path->goal_angle_tolerance : 0.17f);
	const double pi = 3.14159265359;

	// convert the area_map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(path->area_map, sensor_msgs::image_encodings::MONO8);
	const cv::Mat area_map = cv_ptr_obj->image;

	// follow path
	for (size_t i=0; i<path->target_poses.size(); ++i)
	{
		// convert target pose to mira::Pose3
		mira::Pose3 target_pose(Eigen::Vector3f(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z),
				Eigen::Quaternionf(path->target_poses[i].pose.orientation.w, path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z));
		std::cout << "  Next pose: " << target_pose.x() << ", " << target_pose.y() << ", " << target_pose.yaw() << std::endl;

		// publish computed next target
		publishComputedTarget(tf::StampedTransform(tf::Transform(tf::Quaternion(path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z, path->target_poses[i].pose.orientation.w), tf::Vector3(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z)), ros::Time::now(), map_frame_, robot_frame_));

		// check whether the next target pose is accessible (i.e. cost_map < 0.89) and shift occluded targets into accessible space
		const double cost_map_threshold = 0.89;
		{
			boost::mutex::scoped_lock lock(cost_map_mutex_);
			if (cost_map_.getMat().empty() != true)
			{
				const cv::Mat& cost_map = cost_map_.getMat();
				cv::Point target_position_pixel((target_pose.x()+cost_map_.getWorldOffset()[0])/cost_map_.getCellSize(), (target_pose.y()+cost_map_.getWorldOffset()[1])/cost_map_.getCellSize());
				//std::cout << "target_position_pixel: (" << target_position_pixel.x << ", " << target_position_pixel.y << "),   value: " << cost_map.at<double>(target_position_pixel) << std::endl;
				//cv::Mat temp = cost_map.clone();
				//cv::circle(temp, target_position_pixel, 2, cv::Scalar(0.5), -1);
				//cv::imshow("cost_map", temp);
				//cv::waitKey(10);
				if (cost_map.at<double>(target_position_pixel) >= cost_map_threshold)
				{
					std::cout << "  -  Target (" << target_pose.x() << ", " << target_pose.y() << ")=(" << target_position_pixel.x << ", " << target_position_pixel.y << "),   value: " << cost_map.at<double>(target_position_pixel) << "    is not accessible." << std::endl;
					// try to shift target perpendicular to driving direction by at most coverage_radius distance,
					// accept the closest accessible target left or right which is still located within the allowed area of path->area_map
					cv::Point target_position_pixel_left, target_position_pixel_right;
					double distance_target_left = DBL_MAX, distance_target_right = DBL_MAX;
					const cv::Point direction_max_left(-1.2*coverage_radius_/cost_map_.getCellSize()*sin(target_pose.yaw()), 1.2*coverage_radius_/cost_map_.getCellSize()*cos(target_pose.yaw()));
					cv::LineIterator line_left(cost_map, target_position_pixel, target_position_pixel+direction_max_left, 8, false);
					for (int l=0; l<line_left.count; ++l, ++line_left)
					{
						if (*((const double*)*line_left)<cost_map_threshold && area_map.at<uchar>(line_left.pos())==255)
						{
							target_position_pixel_left = line_left.pos();
							distance_target_left = cv::norm(line_left.pos()-target_position_pixel);
							break;
						}
					}
					cv::LineIterator line_right(cost_map, target_position_pixel, target_position_pixel-direction_max_left, 8, false);
					for (int l=0; l<line_right.count; ++l, ++line_right)
					{
						if (*((const double*)*line_right)<cost_map_threshold && area_map.at<uchar>(line_right.pos())==255)
						{
							target_position_pixel_right = line_right.pos();
							distance_target_right = cv::norm(line_right.pos()-target_position_pixel);
							break;
						}
					}

					// evaluate results
					if (distance_target_left>1e20 && distance_target_right>1e20)
					{
						// abort if no accessible pixel was found
						std::cout << "  -> No alternative target found." << std::endl;
						continue;
					}
					else
					{
						// set new target pose
						if (distance_target_left < distance_target_right)
						{
							target_pose.x() = target_position_pixel_left.x*cost_map_.getCellSize() - cost_map_.getWorldOffset()[0];
							target_pose.y() = target_position_pixel_left.y*cost_map_.getCellSize() - cost_map_.getWorldOffset()[1];
							std::cout << "  -> Alternative target (" << target_pose.x() << ", " << target_pose.y() << ")=(" << target_position_pixel_left.x << ", " << target_position_pixel_left.y << "),   value: " << cost_map.at<double>(target_position_pixel_left) << "    found." << std::endl;
						}
						else
						{
							target_pose.x() = target_position_pixel_right.x*cost_map_.getCellSize() - cost_map_.getWorldOffset()[0];
							target_pose.y() = target_position_pixel_right.y*cost_map_.getCellSize() - cost_map_.getWorldOffset()[1];
							std::cout << "  -> Alternative target (" << target_pose.x() << ", " << target_pose.y() << ")=(" << target_position_pixel_right.x << ", " << target_position_pixel_right.y << "),   value: " << cost_map.at<double>(target_position_pixel_right) << "    found." << std::endl;
						}
					}
				}
			}
		}
		// publish commanded next target
		publishCommandedTarget(tf::StampedTransform(tf::Transform(tf::Quaternion(target_pose.r.x(), target_pose.r.y(), target_pose.r.z(), target_pose.r.w()), tf::Vector3(target_pose.x(), target_pose.y(), target_pose.z())), ros::Time::now(), map_frame_, robot_frame_));

		// get current robot speed
		double robot_speed_x = 0.; // [m/s]
		double robot_speed_theta = 0.;	// [rad/s]
		getCurrentRobotSpeed(robot_speed_x, robot_speed_theta);

		// adapt position task accuracy to robot speed -> the faster the robot moves the more accuracy is needed
		const double max_speed = 0.5;
		const double position_accuracy = 0.05 + 0.2 * std::max(0., max_speed-fabs(robot_speed_x))/max_speed;

		// command new navigation goal
		mira::navigation::TaskPtr task(new mira::navigation::Task());
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(target_pose.x(), target_pose.y()),
				/*0.1, 0.1,*/ position_accuracy, position_accuracy, "/GlobalFrame")));	// impose strong precision constraints, otherwise path cannot be followed properly
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::SmoothTransitionTask(true, true)));
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(target_pose.yaw(), 0.087)));	// impose strong precision constraints, otherwise path cannot be followed properly
		//task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::BOTH, 5.0f)));
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 0.9f /*0.98f*/)));	// costs for opposite task, 1.0 is forbidden, 0.0 is cheap/indifferent=BOTH

		// Set this as our goal. Will cause the robot to start driving.
		mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
		r.wait();

		// wait until close to target
		waitForTargetApproach(target_pose, goal_position_tolerance, goal_angle_tolerance);
	}

	std::cout << "  Path following successfully terminated." << std::endl;

	// this sends the response back to the caller
	PathActionServer::Result res;
	path_action_server_->setSucceeded(res);
#else
	ROS_ERROR("ScitosDrive::path_callback: This function is not compiled. Install the MIRA Pilot addon and make sure it is found by cmake.");

	// this sends the response back to the caller
	PathActionServer::Result res;
	path_action_server_->setAborted(res);
#endif
}

// Option with PathFollowTask
//void ScitosDrive::path_callback(const scitos_msgs::MoveBasePathGoalConstPtr& path)
//{
//#ifdef __WITH_PILOT__
//	/*
//	 * https://www.mira-project.org/MIRA-doc/toolboxes/Navigation/classmira_1_1navigation_1_1PathFollowTask.html#_details
//	 * http://www.mira-project.org/MIRA-doc/domains/tutorials/WaypointVisitor/index.html
//	 */
//	ROS_INFO("ScitosDrive::path_callback: Following a path.");
//
//	const float path_tolerance = (path->path_tolerance>0.f ? path->path_tolerance : 0.1f);
//	const float goal_position_tolerance = (path->goal_position_tolerance>0.f ? path->goal_position_tolerance : 0.1f);
//	const float goal_angle_tolerance = (path->goal_angle_tolerance>0.f ? path->goal_angle_tolerance : 0.17f);
//	const double pi = 3.14159265359;
//
//	// visit first pose with normal navigation
//	for (size_t i=0; i<1/*path->target_poses.size()*/; ++i)
//	{
//		// convert target pose to mira::Pose3
//		mira::Pose3 target_pose(Eigen::Vector3f(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z),
//				Eigen::Quaternionf(path->target_poses[i].pose.orientation.w, path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z));
//
//		std::cout << "  Start pose: " << target_pose.x() << ", " << target_pose.y() << ", " << target_pose.yaw() << std::endl;
//
//		// command new navigation goal
//		mira::navigation::TaskPtr task(new mira::navigation::Task());
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(target_pose.x(), target_pose.y()), 0.1, 0.1)));	// impose strong precision constraints, otherwise path cannot be followed properly
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(target_pose.yaw(), 0.087)));	// impose strong precision constraints, otherwise path cannot be followed properly
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::BOTH, 5.0f)));
//
//		// Set this as our goal. Will cause the robot to start driving.
//		mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
//		r.wait();
//
//		// wait until close to target
//		while (true)
//		{
//			mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
//			double distance_to_goal = (target_pose.x()-robot_pose.x())*(target_pose.x()-robot_pose.x()) + (target_pose.y()-robot_pose.y())*(target_pose.y()-robot_pose.y());
//			double delta_angle = normalize_angle(target_pose.yaw()-robot_pose.yaw());
//			//std::cout << "      - current robot pose: " << robot_pose.t(0) << ", " << robot_pose.t(1) << ", " << robot_pose.yaw() << "    dist=" << distance_to_goal << "     phi=" << delta_angle << std::endl;
//			boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);
//			if (nav_pilot_event_status_.compare("PlanAndDrive")!=0 || (distance_to_goal<goal_position_tolerance*goal_position_tolerance && fabs(delta_angle)<goal_angle_tolerance))
//				break;
//			ros::spinOnce();
//		}
//	}
//
//	// start path following behavior for the remaining trajectory
//	mira::navigation::TaskPtr task(new mira::navigation::Task());
//	mira::navigation::PathFollowTask* path_follow_task_ptr = new mira::navigation::PathFollowTask(path_tolerance, goal_position_tolerance, mira::Anglef(mira::Radian<float>(goal_angle_tolerance)));
//	path_follow_task_ptr->frame = "/GlobalFrame";
//	mira::navigation::SubTaskPtr path_follow_task(path_follow_task_ptr);
//	const float translation_sampling_step = 1e10f;//0.1f;	// [m]
//	const float rotation_sampling_step = 1e10f;//0.17f;		// [rad]
//	// set up path command
//	for (size_t i=0; i<path->target_poses.size(); ++i)
//	{
//		// convert target pose to mira::Pose3
//		mira::Pose3 target_pose(Eigen::Vector3f(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z),
//				Eigen::Quaternionf(path->target_poses[i].pose.orientation.w, path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z));
//
//		// append target pose to path vector
//		const mira::Pose2 pose(target_pose.x(), target_pose.y(), target_pose.yaw());
//		// interpolate too large steps in the path
//		if (i>0)
//		{
//			const mira::Pose2& prev_pose = path_follow_task_ptr->points.back();
//			if (fabs(pose.x()-prev_pose.x())>translation_sampling_step || fabs(pose.y()-prev_pose.y())>translation_sampling_step || fabs(normalize_angle(pose.phi()-prev_pose.phi()))>rotation_sampling_step)
//			{
//				int interpolations = std::max<int>(1, (int)fabs(pose.x()-prev_pose.x()) / translation_sampling_step);
//				interpolations = std::max<int>(interpolations, (int)fabs(pose.y()-prev_pose.y()) / translation_sampling_step);
//				const double delta_phi = normalize_angle(pose.phi()-prev_pose.phi());
//				interpolations = std::max<int>(interpolations, (int)fabs(delta_phi) / rotation_sampling_step);
//				const double step_width = 1.0/(interpolations+1.);
//				for (double k=step_width; k<0.9999999; k+=step_width)
//					path_follow_task_ptr->points.push_back(mira::Pose2(prev_pose.x()+k*(pose.x()-prev_pose.x()), prev_pose.y()+k*(pose.y()-prev_pose.y()), prev_pose.phi()+k*delta_phi));
//			}
//		}
//		path_follow_task_ptr->points.push_back(pose);
//	}
//	std::cout << "  Following path with " << path_follow_task_ptr->points.size() << " interpolation points." << std::endl;
////	for (size_t i=0; i<path_follow_task_ptr->points.size(); ++i)
////		std::cout << "  Pose " << i << ": " << path_follow_task_ptr->points[i].x() << ", " << path_follow_task_ptr->points[i].y() << ", " << path_follow_task_ptr->points[i].phi() << std::endl;
//	task->addSubTask(path_follow_task);
//	//task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::BOTH, 0.5f)));
//	// Set this as our goal. Will cause the robot to start driving.
//	mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
//	r.wait();
//
//	// wait until close to target
//	const mira::Pose2& target_pose = path_follow_task_ptr->points.back();
//	ros::Duration(1).sleep();
//	while (true)
//	{
//		mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
//		double distance_to_goal = (target_pose.x()-robot_pose.x())*(target_pose.x()-robot_pose.x()) + (target_pose.y()-robot_pose.y())*(target_pose.y()-robot_pose.y());
//		double delta_angle = normalize_angle(target_pose.phi()-robot_pose.yaw());
//		//std::cout << "      - current robot pose: " << robot_pose.t(0) << ", " << robot_pose.t(1) << ", " << robot_pose.yaw() << "    dist=" << distance_to_goal << "     phi=" << delta_angle << std::endl;
//		boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);
//		if (nav_pilot_event_status_.compare("PlanAndDrive")!=0 || (distance_to_goal<goal_position_tolerance*goal_position_tolerance && fabs(delta_angle)<goal_angle_tolerance))
//			break;
//		ros::spinOnce();
//	}
//	std::cout << "  Path following successfully terminated." << std::endl;
//
//	// this sends the response back to the caller
//	PathActionServer::Result res;
//	path_action_server_->setSucceeded(res);
//#else
//	ROS_ERROR("ScitosDrive::path_callback: This function is not compiled. Install the MIRA Pilot addon and make sure it is found by cmake.");
//
//	// this sends the response back to the caller
//	PathActionServer::Result res;
//	path_action_server_->setAborted(res);
//#endif
//}


// Wall following function
void ScitosDrive::wall_follow_callback(const scitos_msgs::MoveBasePathGoalConstPtr& path)
{
#ifdef __WITH_PILOT__
	ROS_INFO("ScitosDrive::wall_follow_callback: Driving along a wall.");

	// access the cost map for finding the points with a fixed distance to walls
	cv::Mat cost_map;
	cv::Point2d cost_map_offset(0.,0.);	// [m]
	double cost_map_resolution = 0.;	// [m/cell]
	{
		boost::mutex::scoped_lock lock(cost_map_mutex_);
		if (cost_map_.getMat().empty() == false)
		{
			cost_map = cost_map_.getMat().clone();
			cost_map_offset = cv::Point2d(-cost_map_.getWorldOffset()[0], -cost_map_.getWorldOffset()[1]);
			cost_map_resolution = cost_map_.getCellSize();
		}
	}

	if (cost_map.empty() == true)
	{
		ROS_ERROR("ScitosDrive::wall_follow_callback: Could not read the cost map.");

		// this sends the response back to the caller
		PathActionServer::Result res;
		wall_follow_action_server_->setAborted(res);
		return;
	}

	// convert the area_map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(path->area_map, sensor_msgs::image_encodings::MONO8);
	const cv::Mat area_map = cv_ptr_obj->image;
	if (area_map.cols!=cost_map.cols || area_map.rows!=cost_map.rows)
	{
		ROS_ERROR("ScitosDrive::wall_follow_callback: The cost map and the provided area map are different in size.");

		// this sends the response back to the caller
		PathActionServer::Result res;
		wall_follow_action_server_->setAborted(res);
		return;
	}


//	// todo: get with message
//	const int u_min = std::max(0, (int)((-2.-cost_map_offset.x)/cost_map_resolution));
//	const int u_max = std::min(cost_map.cols, (int)((6.3-cost_map_offset.x)/cost_map_resolution));
//	const int v_min = std::max(0, (int)((-4.6-cost_map_offset.y)/cost_map_resolution));
//	const int v_max = std::min(cost_map.rows, (int)((2.8-cost_map_offset.y)/cost_map_resolution));
	std::cout << "cost_map_offset.x=" << cost_map_offset.x << "    cost_map_offset.y=" << cost_map_offset.y << std::endl;
//	std::cout << "u=" << u_min << "," << u_max << "    v=" << v_min << "," << v_max << std::endl;

	// find the points near walls in the accessible area of the room
	cv::Mat level_set_map = cv::Mat::zeros(cost_map.rows, cost_map.cols, CV_8UC1);
	for (int v=0; v<cost_map.rows; ++v)
	{
		for (int u=0; u<cost_map.cols; ++u)
		{
			if (area_map.at<uchar>(v,u)==255 && cost_map.at<double>(v,u) < 1.)
			{
				for (int dv=-1; dv<=1; ++dv)
				{
					for (int du=-1; du<=1; ++du)
					{
						const int nu = u+du;
						const int nv = v+dv;
						if (nu<0 || nu>=cost_map.cols || nv<0 || nv>cost_map.rows)
							continue;
						if (cost_map.at<double>(nv,nu) >= 1.)
							level_set_map.at<uchar>(v,u) = 255;
					}
				}
			}
		}
	}
//	cv::Mat temp = cost_map.clone();
//	for (int v=0; v<cost_map.rows; ++v)
//		for (int u=0; u<cost_map.cols; ++u)
//			if (level_set_map.at<uchar>(v,u) == 255)
//				temp.at<double>(v,u) = 0.;
//	cv::imshow("cost_map", temp);
//	cv::waitKey(10);

	// determine a preferred driving direction for each point
	const double pi = 3.14159265359;
	const double direction_offset = 0.5*pi;		// the direction offset is +pi/2 for wall following on the right side and -pi/2 for wall following on the left side
	cv::Mat cost_map_dx, cost_map_dy;
	cv::Sobel(cost_map, cost_map_dx, CV_64F, 1, 0, 3);
	cv::Sobel(cost_map, cost_map_dy, CV_64F, 0, 1, 3);
	cv::Mat driving_direction(cost_map.rows, cost_map.cols, CV_32FC1);
	for (int v=0; v<level_set_map.rows; ++v)
		for (int u=0; u<level_set_map.cols; ++u)
			if (level_set_map.at<uchar>(v,u) == 255)
				driving_direction.at<float>(v,u) = atan2(cost_map_dy.at<double>(v,u), cost_map_dx.at<double>(v,u)) + direction_offset;

	// find the closest position to the robot pose
	mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
	cv::Point current_pos(0,0);
	double min_dist_sqr = 1e10;
	for (int v=0; v<level_set_map.rows; ++v)
	{
		for (int u=0; u<level_set_map.cols; ++u)
		{
			if (level_set_map.at<uchar>(v,u) == 255)
			{
				cv::Point2d pos(u*cost_map_resolution+cost_map_offset.x, v*cost_map_resolution+cost_map_offset.y);
				const double dist_sqr = (pos.x-robot_pose.x())*(pos.x-robot_pose.x()) + (pos.y-robot_pose.y())*(pos.y-robot_pose.y());
				if (dist_sqr < min_dist_sqr)
				{
					min_dist_sqr = dist_sqr;
					current_pos = cv::Point(u,v);
				}
			}
		}
	}

	// collect the wall following path
	std::vector<cv::Vec3d> wall_poses;
	wall_poses.push_back(cv::Vec3d(current_pos.x*cost_map_resolution+cost_map_offset.x, current_pos.y*cost_map_resolution+cost_map_offset.y, driving_direction.at<float>(current_pos)));
	level_set_map.at<uchar>(current_pos) = 0;
	while (true)
	{
		cv::Point next_pos(-1,-1);
		// try to find a suitable point in the neighborhood
		double max_cos_angle = -1e10;
		const double dd_x = cos(driving_direction.at<float>(current_pos));
		const double dd_y = sin(driving_direction.at<float>(current_pos));
		for (int dv=-1; dv<=1; ++dv)
		{
			for (int du=-1; du<=1; ++du)
			{
				const int nu = current_pos.x+du;
				const int nv = current_pos.y+dv;
				if (nu<0 || nu>=cost_map.cols || nv<0 || nv>cost_map.rows || level_set_map.at<uchar>(nv,nu)!=255)	// the last condition implicitly excludes the center pixel
					continue;

				// determine the angle difference
				double cos_angle = dd_x*du + dd_y*dv;
				if (cos_angle > max_cos_angle)
				{
					max_cos_angle = cos_angle;
					next_pos = cv::Point(nu, nv);
				}
			}
		}

		// if no suitable point was found in the neighborhood search the whole map for the closest next point
		if (next_pos.x < 0)
		{
			double min_dist_sqr = 1e10;
			for (int v=0; v<level_set_map.rows; ++v)
			{
				for (int u=0; u<level_set_map.cols; ++u)
				{
					if (level_set_map.at<uchar>(v,u) == 255)
					{
						const double dist_sqr = (current_pos.x-u)*(current_pos.x-u) + (current_pos.y-v)*(current_pos.y-v);
						if (dist_sqr < min_dist_sqr)
						{
							min_dist_sqr = dist_sqr;
							next_pos = cv::Point(u,v);
						}
					}
				}
			}
		}

		// if still no other point can be found we are done
		if (next_pos.x < 0)
			break;

		// prepare next step
		level_set_map.at<uchar>(next_pos) = 0;
		current_pos = next_pos;
		wall_poses.push_back(cv::Vec3d(current_pos.x*cost_map_resolution+cost_map_offset.x, current_pos.y*cost_map_resolution+cost_map_offset.y, driving_direction.at<float>(current_pos)));
	}

//	// display path
//	std::cout << "printing path" << std::endl;
//	for(size_t step=1; step<wall_poses.size(); ++step)
//	{
//		cv::Mat fov_path_map = cost_map.clone();
//		cv::resize(fov_path_map, fov_path_map, cv::Size(), 2, 2, cv::INTER_LINEAR);
//		if (wall_poses.size() > 0)
//			cv::circle(fov_path_map, 2*cv::Point((wall_poses[0].val[0]-cost_map_offset.x)/cost_map_resolution, (wall_poses[0].val[1]-cost_map_offset.y)/cost_map_resolution), 2, cv::Scalar(0.6), CV_FILLED);
//		for(size_t i=1; i<=step; ++i)
//		{
//			cv::Point p1((wall_poses[i-1].val[0]-cost_map_offset.x)/cost_map_resolution, (wall_poses[i-1].val[1]-cost_map_offset.y)/cost_map_resolution);
//			cv::Point p2((wall_poses[i].val[0]-cost_map_offset.x)/cost_map_resolution, (wall_poses[i].val[1]-cost_map_offset.y)/cost_map_resolution);
//			cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(0.8), CV_FILLED);
//			cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(0.6), 1);
//			cv::Point p3(p2.x+5*cos(wall_poses[i].val[2]), p2.y+5*sin(wall_poses[i].val[2]));
//			if (i==step)
//			{
//				cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(0.3), CV_FILLED);
//				cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(0.6), 1);
//				cv::line(fov_path_map, 2*p2, 2*p3, cv::Scalar(0.2), 1);
//			}
//		}
//		cv::imshow("cell path", fov_path_map);
//		cv::waitKey();
//	}

	// move through the wall poses
	const float path_tolerance = (path->path_tolerance>0.f ? path->path_tolerance : 0.1f);
	const float goal_position_tolerance = (path->goal_position_tolerance>0.f ? path->goal_position_tolerance : 0.1f);
	const float goal_angle_tolerance = (path->goal_angle_tolerance>0.f ? path->goal_angle_tolerance : 0.17f);
	for (std::vector<cv::Vec3d>::iterator pose = wall_poses.begin(); pose!=wall_poses.end(); ++pose)
	{
		std::cout << "  Next pose: " << pose->val[0] << ", " << pose->val[1] << ", " << pose->val[2] << std::endl;

		// todo: mark as visited and do not approach a pose in the vicinity again in future
		// todo: use without wall task on longer distances

		// publish computed next target
		//publishComputedTarget(tf::StampedTransform(tf::Transform(tf::Quaternion(path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z, path->target_poses[i].pose.orientation.w), tf::Vector3(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z)), ros::Time::now(), map_frame_, robot_frame_));


		// get current robot speed
		double robot_speed_x = 0.; // [m/s]
		double robot_speed_theta = 0.;	// [rad/s]
		getCurrentRobotSpeed(robot_speed_x, robot_speed_theta);

		// adapt position task accuracy to robot speed -> the faster the robot moves the more accuracy is needed
		const double max_speed = 0.5;
		const double position_accuracy = 0.05 + 0.2 * std::max(0., max_speed-fabs(robot_speed_x))/max_speed;

		// command new navigation goal
		mira::navigation::TaskPtr task(new mira::navigation::Task());
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(pose->val[0], pose->val[1]),
				/*0.1, 0.1,*/ position_accuracy, position_accuracy, "/GlobalFrame")));	// impose strong precision constraints, otherwise path cannot be followed properly
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::SmoothTransitionTask(true, true)));
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::SmoothTransitionPositionTask(mira::Point2f(pose->val[0], pose->val[1]),
//				/*0.1, 0.1,*/ position_accuracy, position_accuracy, "/GlobalFrame", false, false)));	// impose strong precision constraints, otherwise path cannot be followed properly
		// todo: (last point true optional)
		//task->addSubTask(mira::navigation::SubTaskPtr(
		//	new mira::navigation::SmoothTransitionTask(/*smoothTransition=*/true,
		//	                                           /*allowTransit=*/true)));
		// todo: (last point allowTransit=false)
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(pose->val[2], 0.087)));	// impose strong precision constraints, otherwise path cannot be followed properly
		//task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::BOTH, 5.0f)));
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 0.9f /*0.98f*/)));	// costs for opposite task, 1.0 is forbidden, 0.0 is cheap/indifferent=BOTH
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::WallDistanceTask(0.4, 0.5, mira::navigation::WallDistanceTask::KEEP_RIGHT)));

		// Set this as our goal. Will cause the robot to start driving.
		mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
		r.wait();

		// wait until close to target
		mira::Pose3 target_pose(pose->val[0], pose->val[1], 0., pose->val[2], 0., 0.);
		waitForTargetApproach(target_pose, goal_position_tolerance, goal_angle_tolerance);
//		const double robot_freeze_timeout = 4.;	// [s]
//		ros::Time last_robot_movement = ros::Time::now();
//		while (true)
//		{
//			mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
//			double distance_to_goal = (pose->val[0]-robot_pose.x())*(pose->val[0]-robot_pose.x()) + (pose->val[1]-robot_pose.y())*(pose->val[1]-robot_pose.y());
//			double delta_angle = normalize_angle(pose->val[2]-robot_pose.yaw());
//			//std::cout << "      - current robot pose: " << robot_pose.t(0) << ", " << robot_pose.t(1) << ", " << robot_pose.yaw() << "    dist=" << distance_to_goal << "     phi=" << delta_angle << std::endl;
//			// also abort if the robot does not move for a long time
//			{
//				boost::mutex::scoped_lock lock(odom_msg_mutex_);
//				robot_speed_x = odom_msg_.twist.twist.linear.x;
//				robot_speed_theta = odom_msg_.twist.twist.angular.z;
//			}
//			if (fabs(robot_speed_x) > 0.01 || fabs(robot_speed_theta) > 0.01)
//				last_robot_movement = ros::Time::now();
//			double robot_freeze_time = (ros::Time::now()-last_robot_movement).toSec();
//			//std::cout << "robot_freeze_time" << robot_freeze_time << std::endl;
//			boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);
//			if (nav_pilot_event_status_.compare("PlanAndDrive")!=0 ||
//					(distance_to_goal<goal_position_tolerance*goal_position_tolerance && fabs(delta_angle)<goal_angle_tolerance) ||
//					(robot_freeze_time > robot_freeze_timeout))
//				break;
//			ros::spinOnce();
//		}
	}

	std::cout << "  Wall following successfully terminated." << std::endl;

	// this sends the response back to the caller
	PathActionServer::Result res;
	wall_follow_action_server_->setSucceeded(res);
#else
	ROS_ERROR("ScitosDrive::wall_follow_callback: This function is not compiled. Install the MIRA Pilot addon and make sure it is found by cmake.");

	// this sends the response back to the caller
	PathActionServer::Result res;
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
  if (req.enable == true) 
    return set_mira_param_("MainControlUnit.RearLaser.Enabled","true");
  if (req.enable == false) 
    return set_mira_param_("MainControlUnit.RearLaser.Enabled","false");
  return false;
}

bool ScitosDrive::reset_barrier_stop(scitos_msgs::ResetBarrierStop::Request  &req, scitos_msgs::ResetBarrierStop::Response &res) {
  barrier_status_.barrier_stopped = false;
  return true;
}
