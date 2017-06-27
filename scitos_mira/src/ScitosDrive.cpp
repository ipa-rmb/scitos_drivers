
#include "scitos_mira/ScitosDrive.h"
#include "scitos_mira/ScitosG5.h"

#include <transform/RigidTransform.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt64.h>

#include <opencv2/opencv.hpp>

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
  map_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/static/Map", &ScitosDrive::map_data_callback, this);
#endif
  cmd_vel_subscriber_ = robot_->getRosNode().subscribe("/cmd_vel", 1000, &ScitosDrive::velocity_command_callback,
						       this);
  // mira navigation data types: http://www.mira-project.org/MIRA-doc/domains/robot/SCITOSConfigs/index.html
  move_base_action_server_ = boost::shared_ptr<MoveBaseActionServer>(new MoveBaseActionServer(robot_->getRosNode(), "/move_base", boost::bind(&ScitosDrive::move_base_callback, this, _1), false)); // this initializes the action server; important: always set the last parameter to false
  move_base_action_server_->start();

  path_action_server_ = boost::shared_ptr<PathActionServer>(new PathActionServer(robot_->getRosNode(), "/move_base_path", boost::bind(&ScitosDrive::path_callback, this, _1), false)); // this initializes the action server; important: always set the last parameter to false
  path_action_server_->start();

  
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
	localization_tf.header.frame_id = "/map";
	localization_tf.child_frame_id = "/odom";
	mira::RigidTransform3d map_to_odometry = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/robot/OdometryFrame", "/maps/MapFrame");
	tf::transformEigenToMsg(map_to_odometry, localization_tf.transform);
	// send the transform
	robot_->getTFBroadcaster().sendTransform(localization_tf);
#endif
}

#ifdef __WITH_PILOT__
void ScitosDrive::map_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data)
{
	// convert map to ROS format
	nav_msgs::OccupancyGrid grid_msg;
	grid_msg.header.stamp = ros::Time::now();
	grid_msg.header.frame_id = "map";
	grid_msg.info.resolution = data->value().getCellSize();
	grid_msg.info.width = data->value().width();
	grid_msg.info.height = data->value().height();
	grid_msg.info.origin.position.x = -data->value().getWorldOffset()[0];
	grid_msg.info.origin.position.y = -data->value().getWorldOffset()[1];

	cv::Mat map = data->value().getMat();
	grid_msg.data.resize(map.cols*map.rows);
	int i=0;
	for (int v=0; v<map.rows; ++v)
	{
		unsigned char* map_ptr = map.ptr(v);
		for (int u=0; u<map.cols; ++u, ++i, ++map_ptr)
		{
			char value = (char)((double)*map_ptr*100./255);
			value = (value < 5 ? 0 : value);
			value = (value > 95 ? 100 : value);
			grid_msg.data[i] = value;
		}
	}

	map_pub_.publish(grid_msg);
//	cv::imshow("map", map);
//	cv::waitKey();
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

	// visit first pose with normal navigation
	for (size_t i=0; i<1/*path->target_poses.size()*/; ++i)
	{
		// convert target pose to mira::Pose3
		mira::Pose3 target_pose(Eigen::Vector3f(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z),
				Eigen::Quaternionf(path->target_poses[i].pose.orientation.w, path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z));

		std::cout << "  Start pose: " << target_pose.x() << ", " << target_pose.y() << ", " << target_pose.yaw() << std::endl;

		// command new navigation goal
		mira::navigation::TaskPtr task(new mira::navigation::Task());
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(target_pose.x(), target_pose.y()), 0.1, 0.1)));	// impose strong precision constraints, otherwise path cannot be followed properly
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(target_pose.yaw(), 0.087)));	// impose strong precision constraints, otherwise path cannot be followed properly
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::BOTH, 5.0f)));

		// Set this as our goal. Will cause the robot to start driving.
		mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
		r.wait();

		// wait until close to target
		while (true)
		{
			mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
			double distance_to_goal = (target_pose.x()-robot_pose.x())*(target_pose.x()-robot_pose.x()) + (target_pose.y()-robot_pose.y())*(target_pose.y()-robot_pose.y());
			double delta_angle = normalize_angle(target_pose.yaw()-robot_pose.yaw());
			//std::cout << "      - current robot pose: " << robot_pose.t(0) << ", " << robot_pose.t(1) << ", " << robot_pose.yaw() << "    dist=" << distance_to_goal << "     phi=" << delta_angle << std::endl;
			boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);
			if (nav_pilot_event_status_.compare("PlanAndDrive")!=0 || (distance_to_goal<goal_position_tolerance*goal_position_tolerance && fabs(delta_angle)<goal_angle_tolerance))
				break;
			ros::spinOnce();
		}
	}

	// start path following behavior for the remaining trajectory
	mira::navigation::TaskPtr task(new mira::navigation::Task());
	mira::navigation::PathFollowTask* path_follow_task_ptr = new mira::navigation::PathFollowTask(path_tolerance, goal_position_tolerance, mira::Anglef(mira::Radian<float>(goal_angle_tolerance)));
	path_follow_task_ptr->frame = "/GlobalFrame";
	mira::navigation::SubTaskPtr path_follow_task(path_follow_task_ptr);
	const float translation_sampling_step = 0.1f;	// [m]
	const float rotation_sampling_step = 0.17f;		// [rad]
	// set up path command
	for (size_t i=0; i<path->target_poses.size(); ++i)
	{
		// convert target pose to mira::Pose3
		mira::Pose3 target_pose(Eigen::Vector3f(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z),
				Eigen::Quaternionf(path->target_poses[i].pose.orientation.w, path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z));

		// append target pose to path vector
		const mira::Pose2 pose(target_pose.x(), target_pose.y(), target_pose.yaw());
		// interpolate too large steps in the path
		if (i>0)
		{
			const mira::Pose2& prev_pose = path_follow_task_ptr->points.back();
			if (fabs(pose.x()-prev_pose.x())>translation_sampling_step || fabs(pose.y()-prev_pose.y())>translation_sampling_step || fabs(normalize_angle(pose.phi()-prev_pose.phi()))>rotation_sampling_step)
			{
				int interpolations = std::max<int>(1, (int)fabs(pose.x()-prev_pose.x()) / translation_sampling_step);
				interpolations = std::max<int>(interpolations, (int)fabs(pose.y()-prev_pose.y()) / translation_sampling_step);
				const double delta_phi = normalize_angle(pose.phi()-prev_pose.phi());
				interpolations = std::max<int>(interpolations, (int)fabs(delta_phi) / rotation_sampling_step);
				const double step_width = 1.0/(interpolations+1.);
				for (double k=step_width; k<0.9999999; k+=step_width)
					path_follow_task_ptr->points.push_back(mira::Pose2(prev_pose.x()+k*(pose.x()-prev_pose.x()), prev_pose.y()+k*(pose.y()-prev_pose.y()), prev_pose.phi()+k*delta_phi));
			}
		}
		path_follow_task_ptr->points.push_back(pose);
	}
	std::cout << "  Following path with " << path_follow_task_ptr->points.size() << " interpolation points." << std::endl;
//	for (size_t i=0; i<path_follow_task_ptr->points.size(); ++i)
//		std::cout << "  Pose " << i << ": " << path_follow_task_ptr->points[i].x() << ", " << path_follow_task_ptr->points[i].y() << ", " << path_follow_task_ptr->points[i].phi() << std::endl;
	task->addSubTask(path_follow_task);
	//task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::BOTH, 0.5f)));
	// Set this as our goal. Will cause the robot to start driving.
	mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
	r.wait();

	// wait until close to target
	const mira::Pose2& target_pose = path_follow_task_ptr->points.back();
	ros::Duration(1).sleep();
	while (true)
	{
		mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
		double distance_to_goal = (target_pose.x()-robot_pose.x())*(target_pose.x()-robot_pose.x()) + (target_pose.y()-robot_pose.y())*(target_pose.y()-robot_pose.y());
		double delta_angle = normalize_angle(target_pose.phi()-robot_pose.yaw());
		//std::cout << "      - current robot pose: " << robot_pose.t(0) << ", " << robot_pose.t(1) << ", " << robot_pose.yaw() << "    dist=" << distance_to_goal << "     phi=" << delta_angle << std::endl;
		boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);
		if (nav_pilot_event_status_.compare("PlanAndDrive")!=0 || (distance_to_goal<goal_position_tolerance*goal_position_tolerance && fabs(delta_angle)<goal_angle_tolerance))
			break;
		ros::spinOnce();
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
