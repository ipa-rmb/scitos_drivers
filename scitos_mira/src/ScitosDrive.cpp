
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
#include <cob_srvs/SetInt.h>

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
  map_frame_ = "map";
  robot_frame_ = "base_link";
  map_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>("map_original", 1, true);
  map_clean_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>("map", 1, true);	// todo: hack
  map_segmented_pub_ = robot_->getRosNode().advertise<nav_msgs::OccupancyGrid>("map_segmented", 1, true);
  mira::Channel<mira::maps::OccupancyGrid> map_channel = robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/static/Map", &ScitosDrive::map_data_callback, this);
  robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/cleaning/Map", &ScitosDrive::map_clean_data_callback, this);	// todo: hack:
  robot_->getMiraAuthority().subscribe<mira::maps::OccupancyGrid>("/maps/segmentation/Map", &ScitosDrive::map_segmented_data_callback, this);
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

  // offered services
  robot_->getMiraAuthority().publishService(*this);

  // offered channels
  application_status_sub_ = robot_->getRosNode().subscribe("/application_wet_cleaning_status", 1, &ScitosDrive::application_status_callback, this);
  application_status_channel_ = robot_->getMiraAuthority().publish<int>("AutomaticCleaningStatus");	// todo: hack: put to separate module
#endif
#ifndef DISABLE_MOVEMENTS
  cmd_vel_subscriber_ = robot_->getRosNode().subscribe("/cmd_vel", 1000, &ScitosDrive::velocity_command_callback, this);
#endif
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

// todo: hack just needed by the application
template <typename Reflector>
void ScitosDrive::reflect(Reflector& r)
{
	r.method("start_application", &ScitosDrive::startApplication, this, "This method starts the cleaning application.");
	r.method("start_application_without_cleaning", &ScitosDrive::startApplicationWithoutCleaning, this, "This method starts the cleaning application without using the cleaning device.");
	r.method("pause_application", &ScitosDrive::pauseApplication, this, "This method pauses the cleaning application.");
	r.method("stop_application", &ScitosDrive::stopApplication, this, "This method stops the cleaning application.");
}

// todo: hack: put to separate module
int ScitosDrive::startApplication(void)	// todo: later we should pass a parameter for the service_name of the respective application that shall be started
{
	std::string service_name = "set_application_status_application_wet_cleaning";
	std::cout << ">>>>>>>>>>>>> Starting application." << std::endl;
	robot_->getRosNode().setParam("use_cleaning_device", true);
	cob_srvs::SetIntRequest start_application_req;
	cob_srvs::SetIntResponse start_application_res;
	start_application_req.data = 0;
	ros::service::waitForService(service_name);
	if (ros::service::call(service_name, start_application_req, start_application_res))
	{
		std::cout << "Service call to '" << service_name << "' was successful." << std::endl;
		return 0;
	}
	else
	{
		std::cout << "Service call to '" << service_name << "' was not successful." << std::endl;
		return 1;
	}
	return 1;
}

// todo: hack: put to separate module
int ScitosDrive::startApplicationWithoutCleaning(void)
{
	std::string service_name = "set_application_status_application_wet_cleaning";
	std::cout << ">>>>>>>>>>>>> Starting application without cleaner." << std::endl;
	robot_->getRosNode().setParam("use_cleaning_device", false);
	cob_srvs::SetIntRequest start_application_req;
	cob_srvs::SetIntResponse start_application_res;
	start_application_req.data = 0;
	ros::service::waitForService(service_name);
	if (ros::service::call(service_name, start_application_req, start_application_res))
	{
		std::cout << "Service call to '" << service_name << "' was successful." << std::endl;
		return 0;
	}
	else
	{
		std::cout << "Service call to '" << service_name << "' was not successful." << std::endl;
		return 1;
	}
	return 1;
}

// todo: hack: put to separate module
int ScitosDrive::pauseApplication(void)
{
	std::string service_name = "set_application_status_application_wet_cleaning";
	std::cout << ">>>>>>>>>>>>> Pausing application." << std::endl;
	cob_srvs::SetIntRequest pause_application_req;
	cob_srvs::SetIntResponse pause_application_res;
	pause_application_req.data = 1;
	ros::service::waitForService(service_name);
	if (ros::service::call(service_name, pause_application_req, pause_application_res))
	{
		std::cout << "Service call to '" << service_name << "' was successful." << std::endl;
		return 0;
	}
	else
	{
		std::cout << "Service call to '" << service_name << "' was not successful." << std::endl;
		return 1;
	}
	return 1;
}

// todo: hack: put to separate module
int ScitosDrive::stopApplication(void)
{
	std::string service_name = "set_application_status_application_wet_cleaning";
	std::cout << ">>>>>>>>>>>>> Stopping application." << std::endl;
	cob_srvs::SetIntRequest stop_application_req;
	cob_srvs::SetIntResponse stop_application_res;
	stop_application_req.data = 2;
	ros::service::waitForService(service_name);
	if (ros::service::call(service_name, stop_application_req, stop_application_res))
	{
		std::cout << "Service call to '" << service_name << "' was successful." << std::endl;
		return 0;
	}
	else
	{
		std::cout << "Service call to '" << service_name << "' was not successful." << std::endl;
		return 1;
	}
	return 1;
}

// todo: hack: put to separate module
void ScitosDrive::application_status_callback(const std_msgs::Int32::ConstPtr& msg)
{
	mira::ChannelWrite<int> w = application_status_channel_.write();
	w->timestamp = mira::Time::now(); // optional
	w->value() = msg->data; // oder 1
	w.finish();
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
	try
	{
		mira::RPCFuture<mira::robot::RobotModelPtr> r = robot_->getMiraAuthority().callService<mira::robot::RobotModelPtr>("/robot/Robot", "getRobotModel");
		bool success = r.timedWait(mira::Duration::seconds(10));
		std::cout << "########## Robot Configuration ##########" << std::endl;
		robot_radius_ = 0.325;	// todo:
		coverage_radius_ = 0.25;
		coverage_offset_ = mira::RigidTransform3d(0.29, -0.114, 0.0, 0.0, 0.0, 0.0);
		if (success==true)
		{
			mira::robot::RobotModelPtr robot_model = r.get();
			if (robot_model)
			{
				footprint_ = robot_model->getFootprint();
				robot_radius_ = footprint_.getInnerRadius();
				mira::Footprint coverage_footprint = robot_model->getFootprint("", mira::Time::now(), "CleaningTool");
				coverage_radius_ = 0.5*(coverage_footprint.getOuterRadius()-coverage_footprint.getInnerRadius());	// todo: hack: only works for circle footprints
				coverage_offset_ = robot_->getMiraAuthority().getTransform<mira::RigidTransform3d>("/modules/brushcleaning/BrushFrame", "/robot/RobotFrame");
			}
			else
				std::cout << "Error: could not read robot parameters. Taking standard values." << std::endl;
		}
		else
			std::cout << "Error: could not read robot parameters. Taking standard values." << std::endl;
	}
	catch (std::exception& error)
	{
		std::cout << error.what() << std::endl;
	}
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
	ROS_INFO("ScitosDrive::map_data_callback: Received map.");
	map_world_offset_ = cv::Point2d(data->getWorldOffset()[0], data->getWorldOffset()[1]);
	map_resolution_ = data->getCellSize();
	map_ = data->value();
	publish_grid_map(data->value(), map_pub_, map_frame_);
}

void ScitosDrive::map_clean_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data)
{
	// convert map to ROS format
	ROS_INFO("ScitosDrive::map_clean_data_callback: Received map_clean.");
	publish_grid_map(data->value(), map_clean_pub_, map_frame_);	// todo: hack: using a separately edited map as the "real" map for planning
}

void ScitosDrive::map_segmented_data_callback(mira::ChannelRead<mira::maps::OccupancyGrid> data)
{
	// convert map to ROS format
	ROS_INFO("ScitosDrive::map_segmented_data_callback: Received map_segmented.");
	publish_grid_map(data->value(), map_segmented_pub_, map_frame_);
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

	// wait until arrived at target
	mira::Pose3 target_pose(Eigen::Vector3f(goal->target_pose.pose.position.x, goal->target_pose.pose.position.y, goal->target_pose.pose.position.z),
					Eigen::Quaternionf(goal->target_pose.pose.orientation.w, goal->target_pose.pose.orientation.x, goal->target_pose.pose.orientation.y, goal->target_pose.pose.orientation.z));
	unsigned int return_code = waitForTargetApproach(target_pose, 0.05, 1., 0., false); // TODO (rmb-ma) 0.25 - 3.14 0

	// on robot_freeze try move command again with driving backwards allowed
	if (return_code == 3)
	{
		std::cout << "############################ Cannot drive to goal, trying with driving backwards allowed now." << std::endl;
		task->getSubTask<mira::navigation::PreferredDirectionTask>()->direction = mira::navigation::PreferredDirectionTask::BOTH;
		//task->getSubTask<mira::navigation::PreferredDirectionTask>()->wrongDirectionCost = 0.5f;
		robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task).get();
		return_code = waitForTargetApproach(target_pose, 0.05, 1., 0., false); // wait until close to target
	}

	std::cout << "Stopping move_base_action_server with status " << return_code << std::endl;
	MoveBaseActionServer::Result res;
	if (return_code == 0 || return_code == 2) // todo (rmb-ma). Why return_code == 2 ?
	{
		move_base_action_server_->setSucceeded(res);
	}
	else
	{
		move_base_action_server_->setAborted(res, "move_base_callback returned with status " + return_code);
	}
#else
	ROS_ERROR("ScitosDrive::move_base_callback: This function is not compiled. Install the MIRA Pilot addon and make sure it is found by cmake.");

	// this sends the response back to the caller
	MoveBaseActionServer::Result res;
	move_base_action_server_->setAborted(res);
#endif
}

double ScitosDrive::normalize_angle(double delta_angle)
{
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



float ScitosDrive::computeFootprintToObstacleDistance(const mira::Pose2& target_pose)
{
	mira::Pose2 target_pose_in_merged_map;
	mira::maps::OccupancyGrid merged_map;
	mira::maps::GridMap<float> distance_transformed_map;
	boost::shared_ptr<mira::RigidTransform2f> odometry_to_map;
	return computeFootprintToObstacleDistance(target_pose, target_pose_in_merged_map, merged_map, distance_transformed_map, odometry_to_map);
}

float ScitosDrive::computeFootprintToObstacleDistance(const mira::Pose2& target_pose, mira::Pose2& target_pose_in_merged_map,
		mira::maps::OccupancyGrid& merged_map, mira::maps::GridMap<float>& distance_transformed_map, boost::shared_ptr<mira::RigidTransform2f>& odometry_to_map,
		bool debug_texts)
{
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
		// collision_test_.distanceTransform(merged_map, distance_transformed_map);	// throws error, the code of this function is copied here
		cv::Mat threshold_img;
		cv::threshold(merged_map, threshold_img, 130, 255, CV_THRESH_BINARY_INV);
		cv::distanceTransform(threshold_img, distance_transformed_map, CV_DIST_L2, 5);
		if (debug_texts) std::cout << "distanceTransform done" << std::endl;
//		cv::imshow("merged_map", merged_map);
//		cv::circle(threshold_img, merged_map.world2map(target_pose.t), 2, cv::Scalar(128), -1);
//		cv::imshow("threshold_img", threshold_img);
//		cv::imshow("distance_transformed_map", distance_transformed_map);
//		cv::waitKey(20);
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


int ScitosDrive::waitForTargetApproach(const mira::Pose3& target_pose, const float goal_position_tolerance, const float goal_angle_tolerance, const float cost_map_threshold, const bool path_request)
{
	int return_code = 0;	// 0=goal reached, 1=target inaccessible, 2=pilot not in PlanAndDrive mode, 3=robot does not move
//	cv::Point target_position_pixel(-1,-1);
//	if (cost_map_threshold >= 0)
//	{
//		boost::mutex::scoped_lock lock(cost_map_mutex_);
//		if (cost_map_.getMat().empty() != true)
//			target_position_pixel = cv::Point(target_pose.x()+cost_map_.getWorldOffset()[0]/cost_map_.getCellSize(), target_pose.y()+cost_map_.getWorldOffset()[1]/cost_map_.getCellSize());
//	}
	double robot_freeze_timeout = -1.;	// [s]
	ros::Time last_robot_movement = ros::Time::now();
	while (true)
	{
		if ((path_request && path_action_server_->isPreemptRequested()) || (!path_request && move_base_action_server_->isPreemptRequested()))
		{
			mira::navigation::TaskPtr stopTask(new mira::navigation::Task());
			mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");

			stopTask->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(robot_pose.x(), robot_pose.y()), 0.1f, 0.1f)));
			stopTask->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(robot_pose.yaw(), 0.4)));
			stopTask->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.0f)));

			// Set this as our goal. Will cause the robot to start driving.
			mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", stopTask);
			return 1;
		}

		// check if the target pose is still accessible
		bool target_inaccessible = false;
//		if (cost_map_threshold >= 0 && target_position_pixel.x==-1 && target_position_pixel.y==-1)
//		{
//			boost::mutex::scoped_lock lock(cost_map_mutex_);
//			const cv::Mat& cost_map = cost_map_.getMat();
//			if (cost_map.empty()==false && cost_map.at<double>(target_position_pixel) >= cost_map_threshold)
//				target_inaccessible = true;
//		}
		if (computeFootprintToObstacleDistance(mira::transform_cast<mira::Pose2>(target_pose)) <= 0.f)
			target_inaccessible = true;

		// todo: instead of "/maps/MapFrame" use the frame of the PositionTask (provide PositionTask to this function) for multi level buildings with multiple maps
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
		if (fabs(robot_speed_x) > 0.02 || fabs(robot_speed_theta) > 0.04)
			last_robot_movement = ros::Time::now();
		// todo: check for non-moving condition (robot location)
		double robot_freeze_time = (ros::Time::now()-last_robot_movement).toSec();
		//std::cout << "robot_freeze_time" << robot_freeze_time << std::endl;
		boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);

		if (target_inaccessible || nav_pilot_event_status_.compare("PlanAndDrive") != 0 ||
				(distance_to_goal < goal_position_tolerance && fabs(delta_angle) < goal_angle_tolerance) ||
				(robot_freeze_time > robot_freeze_timeout))
		{
			if (target_inaccessible)
				return_code = 1;
			if (nav_pilot_event_status_.compare("PlanAndDrive") != 0)
				return_code = 2;
			if (robot_freeze_time > robot_freeze_timeout)
				return_code = 3;
			break;
		}
		ros::spinOnce();
	}

	return return_code;
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

mira::Pose2 computeLeftCandidate(const mira::Pose2 &target_pose_in_merged_map, const double offset, cv::Point2d direction_left, const mira::maps::OccupancyGrid &merged_map,
		const mira::RigidTransform2f &map_to_odometry, const cv::Point2d &map_world_offset_, double map_resolution_,
		const double min_obstacle_distance, const cv::Mat &area_map, const mira::model::CollisionTest &collision_test,
		const mira::maps::GridMap<float> &distance_transformed_map, bool &found)
{
	mira::Pose2 pose_right_in_merged_map(target_pose_in_merged_map.x()+offset*direction_left.x, target_pose_in_merged_map.y()+offset*direction_left.y, target_pose_in_merged_map.phi());
	mira::Point2i target_position_pixel = merged_map.world2map(pose_right_in_merged_map.t);
	float obstacle_distance = collision_test.distanceToObstacle(distance_transformed_map, target_position_pixel, target_pose_in_merged_map.phi()) * merged_map.getCellSize(); // in [m]
	mira::Pose2 target_pose_candidate = map_to_odometry * pose_right_in_merged_map;

	//std::cout << "left candidate (" << target_pose_candidate.x() << ", " << target_pose_candidate.y() << ") - " << obstacle_distance << " >= ? " << min_obstacle_distance << std::endl;

	cv::Point map_coordinates = cv::Point((target_pose_candidate.x()+map_world_offset_.x)/map_resolution_, (target_pose_candidate.y()+map_world_offset_.y)/map_resolution_);

	found = obstacle_distance >= min_obstacle_distance && area_map.at<uchar>(map_coordinates)==255;
	return target_pose_candidate;
}

mira::Pose2 computeRightCandidate(const mira::Pose2 &target_pose_in_merged_map, const double offset, cv::Point2d direction_left, const mira::maps::OccupancyGrid &merged_map,
		const mira::RigidTransform2f &map_to_odometry, const cv::Point2d &map_world_offset_, double map_resolution_,
		const double min_obstacle_distance, const cv::Mat &area_map, const mira::model::CollisionTest &collision_test,
		const mira::maps::GridMap<float> &distance_transformed_map, bool &found)
{
	mira::Pose2 pose_right_in_merged_map(target_pose_in_merged_map.x()-offset*direction_left.x, target_pose_in_merged_map.y()-offset*direction_left.y, target_pose_in_merged_map.phi());
	mira::Point2i target_position_pixel = merged_map.world2map(pose_right_in_merged_map.t);
	float obstacle_distance = collision_test.distanceToObstacle(distance_transformed_map, target_position_pixel, target_pose_in_merged_map.phi()) * merged_map.getCellSize(); // in [m]
	mira::Pose2 target_pose_candidate = map_to_odometry * pose_right_in_merged_map;


	//std::cout << "right candidate (" << target_pose_candidate.x() << ", " << target_pose_candidate.y() << ") - " << obstacle_distance << " >= ? " << min_obstacle_distance << std::endl;
	cv::Point map_coordinates = cv::Point((target_pose_candidate.x()+map_world_offset_.x)/map_resolution_, (target_pose_candidate.y()+map_world_offset_.y)/map_resolution_);

	found = obstacle_distance >= min_obstacle_distance && area_map.at<uchar>(map_coordinates)==255;
	return target_pose_candidate;
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

	//const float path_tolerance = (path->path_tolerance>0.f ? path->path_tolerance : 0.1f);
	float goal_position_tolerance = (path->goal_position_tolerance>0.f ? path->goal_position_tolerance : 0.1f);
	const float goal_angle_tolerance = (path->goal_angle_tolerance>0.f ? path->goal_angle_tolerance : 0.17f);

	// convert the area_map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(path->area_map, sensor_msgs::image_encodings::MONO8);
	const cv::Mat area_map = cv_ptr_obj->image;

	// follow path
	for (size_t i=0; i<path->target_poses.size(); ++i)
	{
		mira::Time start_time = mira::Time::now();

		//std::cout << "path_action_server_.isPreemptRequested()=" << path_action_server_->isPreemptRequested() << std::endl;
		if (path_action_server_->isPreemptRequested())
		{
			// this sends the response back to the caller
			PathActionServer::Result res;
			res.last_visited_index = i;
			path_action_server_->setAborted(res);
			return;
		}


		// convert target pose to mira::Pose3
		mira::Pose3 target_pose(Eigen::Vector3f(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z),
				Eigen::Quaternionf(path->target_poses[i].pose.orientation.w, path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z));
		std::cout << "------------------------------------------------------------------------------------------------------------------\n  Next pose: " << target_pose.x() << ", " << target_pose.y() << ", " << target_pose.yaw() << std::endl;


		// publish computed next target
		publishComputedTarget(tf::StampedTransform(tf::Transform(
				tf::Quaternion(path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z, path->target_poses[i].pose.orientation.w), tf::Vector3(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z)), ros::Time::now(), map_frame_, robot_frame_));

		// check distance to closest obstacle
		mira::Pose2 target_pose_in_merged_map;
		mira::maps::OccupancyGrid merged_map;
		mira::maps::GridMap<float> distance_transformed_map;
		boost::shared_ptr<mira::RigidTransform2f> odometry_to_map;
		float obstacle_distance = computeFootprintToObstacleDistance(mira::transform_cast<mira::Pose2>(target_pose), target_pose_in_merged_map, merged_map, distance_transformed_map, odometry_to_map);

		// check whether the next target pose is accessible (i.e. cost_map < 0.89) and shift occluded targets into accessible space
		// now uses the exact footprint for collision checking
		//const double cost_map_threshold = 0.45;	// this is an obstacle distance of inner_circle_radius + 0.1 (min_dist=0.90) + 0.5 (max_dist=0.00) //0.89;
		const double cost_map_threshold = -1.0;	// deactivates the cost map
		const double min_obstacle_distance = 0.02;	// in [m]
//		mira::ChannelRead<mira::maps::OccupancyGrid> merged_map = merged_map_channel_.read(/*mira::Time::now(), mira::Duration::seconds(1)*/); // enforce current data? need exception handling!
		const double max_track_offset = 2.0 * 0.9;		// in [m]	// todo: param

//		std::cout << "merged map: " << merged_map->size() << "  "<<  merged_map->getCellSize() << " " <<  merged_map->getOffset() << std::endl;
//		mira::maps::GridMap<float> distance_transformed_map(merged_map->size(), merged_map->getCellSize(), merged_map->getOffset());
//		std::cout << "distance_transformed_map: " << distance_transformed_map.size() << std::endl;
//		// collision_test_.distanceTransform(merged_map->value(), distance_transformed_map);	// throws error, the code of this function is copied here
//		cv::Mat threshold_img;
//		cv::threshold(merged_map->value(), threshold_img, 140, 255, CV_THRESH_BINARY_INV);
//		cv::distanceTransform(threshold_img, distance_transformed_map, CV_DIST_L2, 5);
//		std::cout << "distanceTransform done" << std::endl;
//
//		mira::RigidTransform2f odometry_to_map = robot_->getMiraAuthority().getTransform<mira::RigidTransform2f>("/maps/MapFrame", "/robot/OdometryFrame");
//		const mira::Pose2 target_pose_in_merged_map = odometry_to_map * mira::transform_cast<mira::Pose2>(target_pose);
//		std::cout << "odometry_to_map : " << odometry_to_map << std::endl;
//		std::cout << "pose : " << target_pose_in_merged_map << std::endl;
//		mira::Point2i target_position_pixel = merged_map->world2map(target_pose_in_merged_map.t);
//		float obstacle_distance = collision_test_.distanceToObstacle(distance_transformed_map, target_position_pixel, target_pose_in_merged_map.phi()) * merged_map->getCellSize(); // in [m]
//		std::cout << "target_position_pixel: (" << target_position_pixel.x() << ", " << target_position_pixel.y() << "),   distance: " << obstacle_distance << std::endl;

		if (obstacle_distance < min_obstacle_distance)
		{
			std::cout << "  -  Target (" << target_pose.x() << ", " << target_pose.y() << "),   distance: " << obstacle_distance << "    is not accessible." << std::endl;
			// try to shift target perpendicular to driving direction,
			// accept the closest accessible target left or right which is still located within the allowed area of path->area_map
			const cv::Point2d direction_left(-sin(target_pose_in_merged_map.phi()), cos(target_pose_in_merged_map.phi()));
			mira::RigidTransform2f map_to_odometry = odometry_to_map->inverse();
			bool alternative_target_found = false;
			for (double offset = merged_map.getCellSize(); offset <= max_track_offset; offset += merged_map.getCellSize())
			{

				const double x = path->target_poses[i].pose.position.x;
				const double y = path->target_poses[i].pose.position.y;
				const double next_x = i < path->target_poses.size() - 1 ? path->target_poses[i+1].pose.position.x : 0;
				const double next_y = i < path->target_poses.size() - 1 ? path->target_poses[i+1].pose.position.y : 0;

				// use of determinant between vectors target_pose and next_pose
				if (x*next_y - y*next_x < 0) // right
				{
					bool found = false;
					mira::Pose2 target_left_candidate = computeLeftCandidate(target_pose_in_merged_map, offset, direction_left, merged_map, map_to_odometry, map_world_offset_, map_resolution_,
							min_obstacle_distance, area_map, collision_test_, distance_transformed_map, found);

					if (found) {
						target_pose = mira::transform_cast<mira::Pose3>(target_left_candidate);
						alternative_target_found = true;
						std::cout << "  -> Alternative target LEFT (" << target_pose.x() << ", " << target_pose.y() << "),   distance: " << obstacle_distance << "    found." << std::endl;
						break;
					}

					mira::Pose2 target_right_candidate = computeRightCandidate(target_pose_in_merged_map, offset, direction_left, merged_map, map_to_odometry, map_world_offset_, map_resolution_,
							min_obstacle_distance, area_map, collision_test_, distance_transformed_map, found);

					if (found) {
						target_pose = mira::transform_cast<mira::Pose3>(target_right_candidate);
						alternative_target_found = true;
						std::cout << "  -> Alternative target RIGHT (" << target_pose.x() << ", " << target_pose.y() << "),   distance: " << obstacle_distance << "    found." << std::endl;
						break;
					}
				}
				else
				{
					bool found = false;
					mira::Pose2 target_right_candidate = computeRightCandidate(target_pose_in_merged_map, offset, direction_left, merged_map, map_to_odometry, map_world_offset_, map_resolution_,
							min_obstacle_distance, area_map, collision_test_, distance_transformed_map, found);

					if (found) {
						target_pose = mira::transform_cast<mira::Pose3>(target_right_candidate);
						alternative_target_found = true;
						std::cout << "  -> Alternative target RIGHT (" << target_pose.x() << ", " << target_pose.y() << "),   distance: " << obstacle_distance << "    found." << std::endl;
						break;
					}

					mira::Pose2 target_left_candidate = computeLeftCandidate(target_pose_in_merged_map, offset, direction_left, merged_map, map_to_odometry, map_world_offset_, map_resolution_,
						min_obstacle_distance, area_map, collision_test_, distance_transformed_map, found);

					if (found) {
						target_pose = mira::transform_cast<mira::Pose3>(target_left_candidate);
						alternative_target_found = true;
						std::cout << "  -> Alternative target LEFT (" << target_pose.x() << ", " << target_pose.y() << "),   distance: " << obstacle_distance << "    found." << std::endl;
						break;
					}
				}


			}
			// abort if no accessible alternative pose was found
			if (alternative_target_found == false)
			{
				std::cout << "  -> No alternative target found." << std::endl;
				continue;
			}

//			boost::mutex::scoped_lock lock(cost_map_mutex_);
//			if (cost_map_.getMat().empty() != true)
//			{
//				const cv::Mat& cost_map = cost_map_.getMat();
//				cv::Point target_position_pixel((target_pose.x()+cost_map_.getWorldOffset()[0])/cost_map_.getCellSize(), (target_pose.y()+cost_map_.getWorldOffset()[1])/cost_map_.getCellSize());
//				//std::cout << "target_position_pixel: (" << target_position_pixel.x << ", " << target_position_pixel.y << "),   value: " << cost_map.at<double>(target_position_pixel) << std::endl;
//				//cv::Mat temp = cost_map.clone();
//				//cv::circle(temp, target_position_pixel, 2, cv::Scalar(0.5), -1);
//				//cv::imshow("cost_map", temp);
//				//cv::waitKey(10);
//				if (cost_map.at<double>(target_position_pixel) >= cost_map_threshold)
//				{
//					std::cout << "  -  Target (" << target_pose.x() << ", " << target_pose.y() << ")=(" << target_position_pixel.x << ", " << target_position_pixel.y << "),   value: " << cost_map.at<double>(target_position_pixel) << "    is not accessible." << std::endl;
//					// try to shift target perpendicular to driving direction by at most coverage_radius distance,
//					// accept the closest accessible target left or right which is still located within the allowed area of path->area_map
//					cv::Point target_position_pixel_left, target_position_pixel_right;
//					double distance_target_left = DBL_MAX, distance_target_right = DBL_MAX;
//					const cv::Point direction_max_left(-2.0*coverage_radius_/cost_map_.getCellSize()*sin(target_pose.yaw()), 2.0*coverage_radius_/cost_map_.getCellSize()*cos(target_pose.yaw()));
//					cv::LineIterator line_left(cost_map, target_position_pixel, target_position_pixel+direction_max_left, 8, false);
//					for (int l=0; l<line_left.count; ++l, ++line_left)
//					{
//						if (*((const double*)*line_left)<cost_map_threshold && area_map.at<uchar>(line_left.pos())==255)
//						{
//							target_position_pixel_left = line_left.pos();
//							distance_target_left = cv::norm(line_left.pos()-target_position_pixel);
//							break;
//						}
//					}
//					cv::LineIterator line_right(cost_map, target_position_pixel, target_position_pixel-direction_max_left, 8, false);
//					for (int l=0; l<line_right.count; ++l, ++line_right)
//					{
//						if (*((const double*)*line_right)<cost_map_threshold && area_map.at<uchar>(line_right.pos())==255)
//						{
//							target_position_pixel_right = line_right.pos();
//							distance_target_right = cv::norm(line_right.pos()-target_position_pixel);
//							break;
//						}
//					}
//
//					// evaluate results
//					if (distance_target_left>1e20 && distance_target_right>1e20)
//					{
//						// abort if no accessible pixel was found
//						std::cout << "  -> No alternative target found." << std::endl;
//						continue;
//					}
//					else
//					{
//						// set new target pose
//						if (distance_target_left < distance_target_right)
//						{
//							target_pose.x() = target_position_pixel_left.x*cost_map_.getCellSize() - cost_map_.getWorldOffset()[0];
//							target_pose.y() = target_position_pixel_left.y*cost_map_.getCellSize() - cost_map_.getWorldOffset()[1];
//							std::cout << "  -> Alternative target (" << target_pose.x() << ", " << target_pose.y() << ")=(" << target_position_pixel_left.x << ", " << target_position_pixel_left.y << "),   value: " << cost_map.at<double>(target_position_pixel_left) << "    found." << std::endl;
//						}
//						else
//						{
//							target_pose.x() = target_position_pixel_right.x*cost_map_.getCellSize() - cost_map_.getWorldOffset()[0];
//							target_pose.y() = target_position_pixel_right.y*cost_map_.getCellSize() - cost_map_.getWorldOffset()[1];
//							std::cout << "  -> Alternative target (" << target_pose.x() << ", " << target_pose.y() << ")=(" << target_position_pixel_right.x << ", " << target_position_pixel_right.y << "),   value: " << cost_map.at<double>(target_position_pixel_right) << "    found." << std::endl;
//						}
//					}
//				}
//			}
		}
		// publish commanded next target
		publishCommandedTarget(tf::StampedTransform(tf::Transform(tf::Quaternion(target_pose.r.x(), target_pose.r.y(), target_pose.r.z(), target_pose.r.w()), tf::Vector3(target_pose.x(), target_pose.y(), target_pose.z())), ros::Time::now(), map_frame_, robot_frame_));

		// get current robot speed
		double robot_speed_x = 0.; // [m/s]
		double robot_speed_theta = 0.;	// [rad/s]
		getCurrentRobotSpeed(robot_speed_x, robot_speed_theta);

		//############################# JOEL STUFF ##############################################
		// i is current pose of complete trajectory? if so just get + j points around current pose i.
		//-> find out how much points, are how much distance (in meter) -> get 3 meters of trajectory and check whether it is a curve or not.
		//bool it_is_a_180_curve = false;
		//double distance_to_check = 3.0; // distance of trajectory, where we check whether it is a curve or not. [m]
		// calculate distance between two poses, to calculate number of poses that are in "distance_to_check".
		//int number_of_poses_to_check = distance_to_check/(Eigen::Vector3f(path->target_poses[i-1].pose.position.x - path->target_poses[i].pose.position.x, path->target_poses[i-1].pose.position.y - path->target_poses[i].pose.position.y, path->target_poses[i-1].pose.position.z - path->target_poses[i].pose.position.z).norm());
		// todo: decrease max_speed_x in curves
//		double last_yaw = target_pose.yaw();
//		double angle_sum = 0.;
//		for (size_t j=i; j<=number_of_poses_to_check; ++j)
//		{
//			mira::Pose3 pose(Eigen::Vector3f(path->target_poses[j].pose.position.x, path->target_poses[j].pose.position.y, path->target_poses[j].pose.position.z),
//					Eigen::Quaternionf(path->target_poses[j].pose.orientation.w, path->target_poses[j].pose.orientation.x, path->target_poses[j].pose.orientation.y, path->target_poses[j].pose.orientation.z));
//			//angles::shortest_angular_distance
//		}

		//############################# JOEL STUFF END ##############################################

		const double max_speed_x = 0.6;//in [m/s] 0.6
		const double max_speed_phi = mira::deg2rad(60.f);	// in [rad/s]

		// adapt position task accuracy to robot speed -> the faster the robot moves the more accuracy is needed
		const double position_accuracy = 0.05 + 0.2 * std::max(0., max_speed_x-fabs(robot_speed_x))/max_speed_x;	// only takes effect if the robot ever reaches the goal exactly instead of being commanded to the next goal already
		// 0.05
		// todo: if there is a big distance between two successive goal positions, decrease the goal tolerance
		goal_position_tolerance = 0.4 + robot_speed_x * desired_planning_ahead_time; // todo (rmb-ma) 0.4 + ...

		// command new navigation goal
		mira::navigation::TaskPtr task(new mira::navigation::Task());
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(target_pose.x(), target_pose.y()),
				/*0.1, 0.1,*/ position_accuracy, position_accuracy, "/maps/MapFrame")));	// impose strong precision constraints, otherwise path cannot be followed properly
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::SmoothTransitionTask(true, true)));
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(target_pose.yaw(), 0.087)));	// impose strong precision constraints, otherwise path cannot be followed properly
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::VelocityTask(mira::Velocity2(max_speed_x, 0.0, max_speed_phi))));	// limit the max allowed speed
		//task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::BOTH, 5.0f)));
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 1.f/*0.9f *//*0.98f*/)));	// costs for opposite task, 1.0 is forbidden, 0.0 is cheap/indifferent=BOTH

		// Set this as our goal. Will cause the robot to start driving.
		mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
		// could do something else here
		r.wait();
		//std::cout << " --- " << (mira::Time::now()-start_time).totalMilliseconds()<< "ms" << std::endl;

		// wait until close to target
		unsigned int return_code = waitForTargetApproach(target_pose, goal_position_tolerance, goal_angle_tolerance, cost_map_threshold, true);
		//const int return_code = waitForTargetApproach(target_pose, 0.02, 0.5, cost_map_threshold);
		//std::cout << "   returned from waitForTargetApproach with return code: " << return_code << std::endl;

		// on robot_freeze try move command again with driving backwards allowed
		if (return_code==3)
		{
			std::cout << "############################ Cannot drive to goal, trying with driving backwards allowed now." << std::endl;
			task->getSubTask<mira::navigation::PreferredDirectionTask>()->direction = mira::navigation::PreferredDirectionTask::BOTH;
			//task->getSubTask<mira::navigation::PreferredDirectionTask>()->wrongDirectionCost = 0.5f;
			robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task).get();
			return_code = waitForTargetApproach(target_pose, goal_position_tolerance, goal_angle_tolerance, cost_map_threshold, true); // wait until close to target
		}

	}

	std::cout << "  Path following successfully terminated." << std::endl;

	// this sends the response back to the caller
	PathActionServer::Result res;

	res.last_visited_index = path->target_poses.size();
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
void ScitosDrive::wall_follow_callback(const scitos_msgs::MoveBaseWallFollowGoalConstPtr& goal)
{
#ifdef __WITH_PILOT__
	ROS_INFO("ScitosDrive::wall_follow_callback: Driving along a wall.");

	const double map_resolution = goal->map_resolution;	// in [m/cell]
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
	const double target_wall_distance_px = 0.55/map_resolution; // todo: robot_radius_/map_resolution;	// target distance of robot center to wall todo: check if valid
	const double target_wall_distance_px_epsilon = 1;			// allowed deviation from target distance of robot center to wall, used for sampling goal poses along the wall

	// convert the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->map, sensor_msgs::image_encodings::MONO8);
	const cv::Mat map = cv_ptr_obj->image;

	// convert the area_map msg in cv format
	cv_ptr_obj = cv_bridge::toCvCopy(goal->area_map, sensor_msgs::image_encodings::MONO8);
	const cv::Mat area_map = cv_ptr_obj->image;

	// convert the coverage_map msg in cv format
	cv_ptr_obj = cv_bridge::toCvCopy(goal->coverage_map, sensor_msgs::image_encodings::MONO8);
	const cv::Mat coverage_map = cv_ptr_obj->image;

	// get the distance-transformed map
	//cv::erode(map, temporary_map, cv::Mat());
	cv::Mat distance_map;	//variable for the distance-transformed map, type: CV_32FC1
	cv::distanceTransform(area_map, distance_map, CV_DIST_L2, 5);
	cv::Mat distance_map_disp;
	cv::convertScaleAbs(distance_map, distance_map_disp);	// conversion to 8 bit image
//	cv::imshow("distance_map_disp", distance_map_disp);
//	cv::waitKey();

//	// reduce the distance transformed map to the current area
//	for (int v=0; v<distance_map.rows; ++v)
//		for (int u=0; u<distance_map.cols; ++u)
//			if (area_map.at<uchar>(v,u)==0)
//				distance_map.at<float>(v,u) = 0.f;
//	cv::convertScaleAbs(distance_map, distance_map_disp);	// conversion to 8 bit image
//	cv::imshow("distance_map_area_disp", distance_map_disp);
//	cv::waitKey();

	// find the points near walls in the accessible area of the room
	cv::Mat level_set_map = cv::Mat::zeros(distance_map.rows, distance_map.cols, CV_8UC1);
	for (int v = 0; v < distance_map.rows; ++v)
		for (int u = 0; u < distance_map.cols; ++u)
			if (fabs(distance_map.at<float>(v,u)-target_wall_distance_px) < target_wall_distance_px_epsilon)
				level_set_map.at<uchar>(v,u) = 255;
//	cv::imshow("level_set_map", level_set_map);
//	cv::waitKey(20);

	// determine a preferred driving direction for each point
	const double direction_offset = -0.5*pi;		// todo: param: the direction offset is -pi/2 for wall following on the right side and +pi/2 for wall following on the left side
	cv::Mat distance_map_dx, distance_map_dy;
	cv::Sobel(distance_map_disp, distance_map_dx, CV_64F, 1, 0, 3);
	cv::Sobel(distance_map_disp, distance_map_dy, CV_64F, 0, 1, 3);
	cv::Mat driving_direction(distance_map.rows, distance_map.cols, CV_32FC1);
	for (int v=0; v<distance_map.rows; ++v)
		for (int u=0; u<distance_map.cols; ++u)
			if (level_set_map.at<uchar>(v,u) == 255)
				driving_direction.at<float>(v,u) = normalize_angle(atan2(distance_map_dy.at<double>(v,u), distance_map_dx.at<double>(v,u)) + direction_offset);

	// find the closest position to the robot pose
	mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
	cv::Point current_pos(0,0);
	double min_dist_sqr = 1e10;
	for (int v=0; v<distance_map.rows; ++v)
	{
		for (int u=0; u<distance_map.cols; ++u)
		{
			if (level_set_map.at<uchar>(v,u) == 255)
			{
				cv::Point2d pos(u*map_resolution+map_origin.x, v*map_resolution+map_origin.y);
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
	std::vector<cv::Vec3d> wall_poses_dense, wall_poses;
	wall_poses_dense.push_back(cv::Vec3d(current_pos.x*map_resolution+map_origin.x, current_pos.y*map_resolution+map_origin.y, driving_direction.at<float>(current_pos)));
	level_set_map.at<uchar>(current_pos) = 0;
	cv::Mat visited_map = -1e11*cv::Mat::ones(level_set_map.rows, level_set_map.cols, CV_32FC1);	// used to mask all cells in the neighborhood of already visited cells
	                                                                                                // write driving direction into visited_map and only skip if driving direction is similar
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
				if (nu<0 || nu>=level_set_map.cols || nv<0 || nv>level_set_map.rows || level_set_map.at<uchar>(nv,nu)!=255)	// the last condition implicitly excludes the center pixel
					continue;

				level_set_map.at<uchar>(nv,nu) = 0;		// mark all neighboring points as visited

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
		if (visited_map.at<float>(next_pos) < -1e10 ||		// do not visit places a second time
				fabs(normalize_angle(driving_direction.at<float>(next_pos)-visited_map.at<float>(next_pos))) > 100./180.*pi)		// except with very different driving direction
		{
			wall_poses_dense.push_back(cv::Vec3d(next_pos.x*map_resolution+map_origin.x, next_pos.y*map_resolution+map_origin.y, driving_direction.at<float>(next_pos)));
			cv::circle(visited_map, next_pos, 3, cv::Scalar(driving_direction.at<float>(next_pos)), -1);
		}
	}
	// reduce density of wall_poses
	wall_poses.push_back(wall_poses_dense[0]);
	size_t last_used_pose_index = 0;
	for (size_t i=1; i<wall_poses_dense.size(); ++i)
	{
		const cv::Vec3d& p0 = wall_poses_dense[last_used_pose_index];
		const cv::Vec3d& p1 = wall_poses_dense[i];
		if ((p1.val[0]-p0.val[0])*(p1.val[0]-p0.val[0]) + (p1.val[1]-p0.val[1])*(p1.val[1]-p0.val[1]) > 0.16*0.16 || normalize_angle(p1.val[2]-p0.val[2]) > 1.5708)
		{
			wall_poses.push_back(wall_poses_dense[i]);
			last_used_pose_index = i;
		}
	}

	// display path
	if (false)
	{
		std::cout << "printing path" << std::endl;
		for(size_t step=wall_poses.size()-1; step<wall_poses.size(); ++step)
		{
			cv::Mat fov_path_map = area_map.clone();
			cv::resize(fov_path_map, fov_path_map, cv::Size(), 2, 2, cv::INTER_LINEAR);
			if (wall_poses.size() > 0)
				cv::circle(fov_path_map, 2*cv::Point((wall_poses[0].val[0]-map_origin.x)/map_resolution, (wall_poses[0].val[1]-map_origin.y)/map_resolution), 2, cv::Scalar(0.6), CV_FILLED);
			for(size_t i=1; i<=step; ++i)
			{
				cv::Point p1((wall_poses[i-1].val[0]-map_origin.x)/map_resolution, (wall_poses[i-1].val[1]-map_origin.y)/map_resolution);
				cv::Point p2((wall_poses[i].val[0]-map_origin.x)/map_resolution, (wall_poses[i].val[1]-map_origin.y)/map_resolution);
				cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(0.8), CV_FILLED);
				cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(0.6), 1);
				if (i==step)
				{
					cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(0.3), CV_FILLED);
					cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(0.6), 1);
					cv::Point p3(p2.x+5*cos(wall_poses[i].val[2]), p2.y+5*sin(wall_poses[i].val[2]));
					cv::line(fov_path_map, 2*p2, 2*p3, cv::Scalar(0.2), 1);
				}
			}
			if (step == wall_poses.size()-1)
			{
				cv::imshow("cell path", fov_path_map);
				cv::waitKey(20);
			}
		}
	}

	// move through the wall poses
	const float path_tolerance = (goal->path_tolerance>0.f ? goal->path_tolerance : 0.1f);
	const float goal_position_tolerance = (goal->goal_position_tolerance>0.f ? goal->goal_position_tolerance : 0.1f);
	const float goal_angle_tolerance = (goal->goal_angle_tolerance>0.f ? goal->goal_angle_tolerance : 0.17f);
	const float target_wall_distance = (goal->target_wall_distance>=0.f ? goal->target_wall_distance : 0.1f);	// target distance between robot and wall during wall following, in [m]
	const float wall_following_off_traveling_distance_threshold = (goal->wall_following_off_traveling_distance_threshold>=0.f ? goal->wall_following_off_traveling_distance_threshold : 1.0f);		// when traveling farther than this threshold distance, the robot does not use the wall following objective, in [m]
	for (std::vector<cv::Vec3d>::iterator pose = wall_poses.begin(); pose!=wall_poses.end(); ++pose)
	{
		std::cout << "wall_follow_action_server_.isPreemptRequested()=" << wall_follow_action_server_->isPreemptRequested() << std::endl;
		if (wall_follow_action_server_->isPreemptRequested())
		{
			// this sends the response back to the caller
			WallFollowActionServer::Result res;
			wall_follow_action_server_->setAborted(res);
			return;
		}

		std::cout << "  Next pose: " << pose->val[0] << ", " << pose->val[1] << ", " << pose->val[2] << std::endl;

		// convert target pose to mira::Pose3
		mira::Pose3 target_pose(pose->val[0], pose->val[1], 0., pose->val[2], 0., 0.);

		// publish computed next target
		//Eigen::Quaternion q = mira::quaternionFromYawPitchRoll(pose->val[2], 0., 0.);
		publishComputedTarget(tf::StampedTransform(tf::Transform(tf::Quaternion(target_pose.r.x(), target_pose.r.y(), target_pose.r.z(), target_pose.r.w()), tf::Vector3(target_pose.x(), target_pose.y(), target_pose.z())), ros::Time::now(), map_frame_, robot_frame_));

		// publish commanded next target
		publishCommandedTarget(tf::StampedTransform(tf::Transform(tf::Quaternion(target_pose.r.x(), target_pose.r.y(), target_pose.r.z(), target_pose.r.w()), tf::Vector3(target_pose.x(), target_pose.y(), target_pose.z())), ros::Time::now(), map_frame_, robot_frame_));

		// get current robot speed
		double robot_speed_x = 0.; // [m/s]
		double robot_speed_theta = 0.;	// [rad/s]
		getCurrentRobotSpeed(robot_speed_x, robot_speed_theta);

		// adapt position task accuracy to robot speed -> the faster the robot moves the more accuracy is needed
		const double max_speed = 0.3;
		const double position_accuracy = 0.05 + 0.2 * std::max(0., max_speed-fabs(robot_speed_x))/max_speed;

		// command new navigation goal
		mira::navigation::TaskPtr task(new mira::navigation::Task());
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(pose->val[0], pose->val[1]),
				/*0.1, 0.1,*/ position_accuracy, position_accuracy, "/maps/MapFrame")));	// impose strong precision constraints, otherwise path cannot be followed properly
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::SmoothTransitionTask(true, true)));
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::SmoothTransitionPositionTask(mira::Point2f(pose->val[0], pose->val[1]),
//				/*0.1, 0.1,*/ position_accuracy, position_accuracy, "/GlobalFrame", false, false)));	// impose strong precision constraints, otherwise path cannot be followed properly
		// todo: (last point true optional)
		//task->addSubTask(mira::navigation::SubTaskPtr(
		//	new mira::navigation::SmoothTransitionTask(/*smoothTransition=*/true,
		//	                                           /*allowTransit=*/true)));
		// todo: (last point allowTransit=false)
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(pose->val[2], 1.5708)));
		//task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::BOTH, 5.0f)));
		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 0.9f /*0.98f*/)));	// costs for opposite task, 1.0 is forbidden, 0.0 is cheap/indifferent=BOTH

		// use without wall task on longer distances or on targets in free space far from a wall
		mira::Pose3 current_robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
		const double distance_to_last_pose_sqr = (pose->val[0]-current_robot_pose.x())*(pose->val[0]-current_robot_pose.x()) + (pose->val[1]-current_robot_pose.y())*(pose->val[1]-current_robot_pose.y());
		if (distance_to_last_pose_sqr <= wall_following_off_traveling_distance_threshold*wall_following_off_traveling_distance_threshold &&
				computeFootprintToObstacleDistance(mira::transform_cast<mira::Pose2>(target_pose)) < std::max(0.2f, 2*target_wall_distance))		// todo: param 0.2 as minimum distance for activating wall following
		{
			std::cout << "ScitosDrive::wall_follow_callback: WallDistanceTask::KEEP_RIGHT active." << std::endl;
			task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::WallDistanceTask(target_wall_distance, 1.0, mira::navigation::WallDistanceTask::KEEP_RIGHT)));
		}

		// Set this as our goal. Will cause the robot to start driving.
		mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
		r.wait();

		// wait until close to target
		const int return_code = waitForTargetApproach(target_pose, goal_position_tolerance, goal_angle_tolerance, -1., false);
		std::cout << "   returned from waitForTargetApproach with return code: " << return_code << std::endl;

		// on robot_freeze try move command again with driving backwards allowed
		if (return_code==3)
		{
			std::cout << "############################ Cannot drive to goal, trying with driving backwards allowed now." << std::endl;
			task->getSubTask<mira::navigation::PreferredDirectionTask>()->direction = mira::navigation::PreferredDirectionTask::BOTH;
			//task->getSubTask<mira::navigation::PreferredDirectionTask>()->wrongDirectionCost = 0.5f;
			robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task).get();
			waitForTargetApproach(target_pose, goal_position_tolerance, goal_angle_tolerance, -1., false); // wait until close to target
		}
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

// Old wall following function
//void ScitosDrive::wall_follow_callback(const scitos_msgs::MoveBaseGoalConstPtr& path)
//{
//#ifdef __WITH_PILOT__
//	ROS_INFO("ScitosDrive::wall_follow_callback: Driving along a wall.");
//
//	// access the cost map for finding the points with a fixed distance to walls
//	cv::Mat cost_map;
//	cv::Point2d cost_map_offset(0.,0.);	// [m]
//	double cost_map_resolution = 0.;	// [m/cell]
//	{
//		boost::mutex::scoped_lock lock(cost_map_mutex_);
//		if (cost_map_.getMat().empty() == false)
//		{
//			cost_map = cost_map_.getMat().clone();
//			cost_map_offset = cv::Point2d(-cost_map_.getWorldOffset()[0], -cost_map_.getWorldOffset()[1]);
//			cost_map_resolution = cost_map_.getCellSize();
//		}
//	}
//
//	if (cost_map.empty() == true)
//	{
//		ROS_ERROR("ScitosDrive::wall_follow_callback: Could not read the cost map.");
//
//		// this sends the response back to the caller
//		PathActionServer::Result res;
//		wall_follow_action_server_->setAborted(res);
//		return;
//	}
//
//	// convert the area_map msg in cv format
//	cv_bridge::CvImagePtr cv_ptr_obj;
//	cv_ptr_obj = cv_bridge::toCvCopy(path->area_map, sensor_msgs::image_encodings::MONO8);
//	const cv::Mat area_map = cv_ptr_obj->image;
//	if (area_map.cols!=cost_map.cols || area_map.rows!=cost_map.rows)
//	{
//		ROS_ERROR("ScitosDrive::wall_follow_callback: The cost map and the provided area map are different in size.");
//
//		// this sends the response back to the caller
//		PathActionServer::Result res;
//		wall_follow_action_server_->setAborted(res);
//		return;
//	}
//
//
////	// todo: get with message
////	const int u_min = std::max(0, (int)((-2.-cost_map_offset.x)/cost_map_resolution));
////	const int u_max = std::min(cost_map.cols, (int)((6.3-cost_map_offset.x)/cost_map_resolution));
////	const int v_min = std::max(0, (int)((-4.6-cost_map_offset.y)/cost_map_resolution));
////	const int v_max = std::min(cost_map.rows, (int)((2.8-cost_map_offset.y)/cost_map_resolution));
//	std::cout << "cost_map_offset.x=" << cost_map_offset.x << "    cost_map_offset.y=" << cost_map_offset.y << std::endl;
////	std::cout << "u=" << u_min << "," << u_max << "    v=" << v_min << "," << v_max << std::endl;
//
//	// find the points near walls in the accessible area of the room
//	cv::Mat level_set_map = cv::Mat::zeros(cost_map.rows, cost_map.cols, CV_8UC1);
//	for (int v=0; v<cost_map.rows; ++v)
//	{
//		for (int u=0; u<cost_map.cols; ++u)
//		{
//			if (area_map.at<uchar>(v,u)==255 && cost_map.at<double>(v,u) < 1.)
//			{
//				for (int dv=-1; dv<=1; ++dv)
//				{
//					for (int du=-1; du<=1; ++du)
//					{
//						const int nu = u+du;
//						const int nv = v+dv;
//						if (nu<0 || nu>=cost_map.cols || nv<0 || nv>cost_map.rows)
//							continue;
//						if (cost_map.at<double>(nv,nu) >= 1.)
//							level_set_map.at<uchar>(v,u) = 255;
//					}
//				}
//			}
//		}
//	}
////	cv::Mat temp = cost_map.clone();
////	for (int v=0; v<cost_map.rows; ++v)
////		for (int u=0; u<cost_map.cols; ++u)
////			if (level_set_map.at<uchar>(v,u) == 255)
////				temp.at<double>(v,u) = 0.;
////	cv::imshow("cost_map", temp);
////	cv::waitKey(10);
//
//	// determine a preferred driving direction for each point
//	const double direction_offset = 0.5*pi;		// the direction offset is +pi/2 for wall following on the right side and -pi/2 for wall following on the left side
//	cv::Mat cost_map_dx, cost_map_dy;
//	cv::Sobel(cost_map, cost_map_dx, CV_64F, 1, 0, 3);
//	cv::Sobel(cost_map, cost_map_dy, CV_64F, 0, 1, 3);
//	cv::Mat driving_direction(cost_map.rows, cost_map.cols, CV_32FC1);
//	for (int v=0; v<level_set_map.rows; ++v)
//		for (int u=0; u<level_set_map.cols; ++u)
//			if (level_set_map.at<uchar>(v,u) == 255)
//				driving_direction.at<float>(v,u) = atan2(cost_map_dy.at<double>(v,u), cost_map_dx.at<double>(v,u)) + direction_offset;
//
//	// find the closest position to the robot pose
//	mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
//	cv::Point current_pos(0,0);
//	double min_dist_sqr = 1e10;
//	for (int v=0; v<level_set_map.rows; ++v)
//	{
//		for (int u=0; u<level_set_map.cols; ++u)
//		{
//			if (level_set_map.at<uchar>(v,u) == 255)
//			{
//				cv::Point2d pos(u*cost_map_resolution+cost_map_offset.x, v*cost_map_resolution+cost_map_offset.y);
//				const double dist_sqr = (pos.x-robot_pose.x())*(pos.x-robot_pose.x()) + (pos.y-robot_pose.y())*(pos.y-robot_pose.y());
//				if (dist_sqr < min_dist_sqr)
//				{
//					min_dist_sqr = dist_sqr;
//					current_pos = cv::Point(u,v);
//				}
//			}
//		}
//	}
//
//	// collect the wall following path
//	std::vector<cv::Vec3d> wall_poses;
//	wall_poses.push_back(cv::Vec3d(current_pos.x*cost_map_resolution+cost_map_offset.x, current_pos.y*cost_map_resolution+cost_map_offset.y, driving_direction.at<float>(current_pos)));
//	level_set_map.at<uchar>(current_pos) = 0;
//	while (true)
//	{
//		cv::Point next_pos(-1,-1);
//		// try to find a suitable point in the neighborhood
//		double max_cos_angle = -1e10;
//		const double dd_x = cos(driving_direction.at<float>(current_pos));
//		const double dd_y = sin(driving_direction.at<float>(current_pos));
//		for (int dv=-1; dv<=1; ++dv)
//		{
//			for (int du=-1; du<=1; ++du)
//			{
//				const int nu = current_pos.x+du;
//				const int nv = current_pos.y+dv;
//				if (nu<0 || nu>=cost_map.cols || nv<0 || nv>cost_map.rows || level_set_map.at<uchar>(nv,nu)!=255)	// the last condition implicitly excludes the center pixel
//					continue;
//
//				// determine the angle difference
//				double cos_angle = dd_x*du + dd_y*dv;
//				if (cos_angle > max_cos_angle)
//				{
//					max_cos_angle = cos_angle;
//					next_pos = cv::Point(nu, nv);
//				}
//			}
//		}
//
//		// if no suitable point was found in the neighborhood search the whole map for the closest next point
//		if (next_pos.x < 0)
//		{
//			double min_dist_sqr = 1e10;
//			for (int v=0; v<level_set_map.rows; ++v)
//			{
//				for (int u=0; u<level_set_map.cols; ++u)
//				{
//					if (level_set_map.at<uchar>(v,u) == 255)
//					{
//						const double dist_sqr = (current_pos.x-u)*(current_pos.x-u) + (current_pos.y-v)*(current_pos.y-v);
//						if (dist_sqr < min_dist_sqr)
//						{
//							min_dist_sqr = dist_sqr;
//							next_pos = cv::Point(u,v);
//						}
//					}
//				}
//			}
//		}
//
//		// if still no other point can be found we are done
//		if (next_pos.x < 0)
//			break;
//
//		// prepare next step
//		level_set_map.at<uchar>(next_pos) = 0;
//		current_pos = next_pos;
//		wall_poses.push_back(cv::Vec3d(current_pos.x*cost_map_resolution+cost_map_offset.x, current_pos.y*cost_map_resolution+cost_map_offset.y, driving_direction.at<float>(current_pos)));
//	}
//
////	// display path
////	std::cout << "printing path" << std::endl;
////	for(size_t step=1; step<wall_poses.size(); ++step)
////	{
////		cv::Mat fov_path_map = cost_map.clone();
////		cv::resize(fov_path_map, fov_path_map, cv::Size(), 2, 2, cv::INTER_LINEAR);
////		if (wall_poses.size() > 0)
////			cv::circle(fov_path_map, 2*cv::Point((wall_poses[0].val[0]-cost_map_offset.x)/cost_map_resolution, (wall_poses[0].val[1]-cost_map_offset.y)/cost_map_resolution), 2, cv::Scalar(0.6), CV_FILLED);
////		for(size_t i=1; i<=step; ++i)
////		{
////			cv::Point p1((wall_poses[i-1].val[0]-cost_map_offset.x)/cost_map_resolution, (wall_poses[i-1].val[1]-cost_map_offset.y)/cost_map_resolution);
////			cv::Point p2((wall_poses[i].val[0]-cost_map_offset.x)/cost_map_resolution, (wall_poses[i].val[1]-cost_map_offset.y)/cost_map_resolution);
////			cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(0.8), CV_FILLED);
////			cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(0.6), 1);
////			cv::Point p3(p2.x+5*cos(wall_poses[i].val[2]), p2.y+5*sin(wall_poses[i].val[2]));
////			if (i==step)
////			{
////				cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(0.3), CV_FILLED);
////				cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(0.6), 1);
////				cv::line(fov_path_map, 2*p2, 2*p3, cv::Scalar(0.2), 1);
////			}
////		}
////		cv::imshow("cell path", fov_path_map);
////		cv::waitKey();
////	}
//
//	// move through the wall poses
//	const float path_tolerance = (path->path_tolerance>0.f ? path->path_tolerance : 0.1f);
//	const float goal_position_tolerance = (path->goal_position_tolerance>0.f ? path->goal_position_tolerance : 0.1f);
//	const float goal_angle_tolerance = (path->goal_angle_tolerance>0.f ? path->goal_angle_tolerance : 0.17f);
//	for (std::vector<cv::Vec3d>::iterator pose = wall_poses.begin(); pose!=wall_poses.end(); ++pose)
//	{
//		std::cout << "  Next pose: " << pose->val[0] << ", " << pose->val[1] << ", " << pose->val[2] << std::endl;
//
//		// todo: mark as visited and do not approach a pose in the vicinity again in future
//		// todo: use without wall task on longer distances
//
//		// publish computed next target
//		//publishComputedTarget(tf::StampedTransform(tf::Transform(tf::Quaternion(path->target_poses[i].pose.orientation.x, path->target_poses[i].pose.orientation.y, path->target_poses[i].pose.orientation.z, path->target_poses[i].pose.orientation.w), tf::Vector3(path->target_poses[i].pose.position.x, path->target_poses[i].pose.position.y, path->target_poses[i].pose.position.z)), ros::Time::now(), map_frame_, robot_frame_));
//
//
//		// get current robot speed
//		double robot_speed_x = 0.; // [m/s]
//		double robot_speed_theta = 0.;	// [rad/s]
//		getCurrentRobotSpeed(robot_speed_x, robot_speed_theta);
//
//		// adapt position task accuracy to robot speed -> the faster the robot moves the more accuracy is needed
//		const double max_speed = 0.5;
//		const double position_accuracy = 0.05 + 0.2 * std::max(0., max_speed-fabs(robot_speed_x))/max_speed;
//
//		// command new navigation goal
//		mira::navigation::TaskPtr task(new mira::navigation::Task());
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PositionTask(mira::Point2f(pose->val[0], pose->val[1]),
//				/*0.1, 0.1,*/ position_accuracy, position_accuracy, "/maps/MapFrame")));	// impose strong precision constraints, otherwise path cannot be followed properly
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::SmoothTransitionTask(true, true)));
////		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::SmoothTransitionPositionTask(mira::Point2f(pose->val[0], pose->val[1]),
////				/*0.1, 0.1,*/ position_accuracy, position_accuracy, "/GlobalFrame", false, false)));	// impose strong precision constraints, otherwise path cannot be followed properly
//		// todo: (last point true optional)
//		//task->addSubTask(mira::navigation::SubTaskPtr(
//		//	new mira::navigation::SmoothTransitionTask(/*smoothTransition=*/true,
//		//	                                           /*allowTransit=*/true)));
//		// todo: (last point allowTransit=false)
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::OrientationTask(pose->val[2], 0.087)));	// impose strong precision constraints, otherwise path cannot be followed properly
//		//task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::BOTH, 5.0f)));
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::PreferredDirectionTask(mira::navigation::PreferredDirectionTask::FORWARD, 0.9f /*0.98f*/)));	// costs for opposite task, 1.0 is forbidden, 0.0 is cheap/indifferent=BOTH
//		task->addSubTask(mira::navigation::SubTaskPtr(new mira::navigation::WallDistanceTask(0.4, 0.5, mira::navigation::WallDistanceTask::KEEP_RIGHT)));
//
//		// Set this as our goal. Will cause the robot to start driving.
//		mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/navigation/Pilot", "setTask", task);
//		r.wait();
//
//		// wait until close to target
//		mira::Pose3 target_pose(pose->val[0], pose->val[1], 0., pose->val[2], 0., 0.);
//		waitForTargetApproach(target_pose, goal_position_tolerance, goal_angle_tolerance);
////		const double robot_freeze_timeout = 4.;	// [s]
////		ros::Time last_robot_movement = ros::Time::now();
////		while (true)
////		{
////			mira::Pose3 robot_pose = robot_->getMiraAuthority().getTransform<mira::Pose3>("/robot/RobotFrame", "/maps/MapFrame");
////			double distance_to_goal = (pose->val[0]-robot_pose.x())*(pose->val[0]-robot_pose.x()) + (pose->val[1]-robot_pose.y())*(pose->val[1]-robot_pose.y());
////			double delta_angle = normalize_angle(pose->val[2]-robot_pose.yaw());
////			//std::cout << "      - current robot pose: " << robot_pose.t(0) << ", " << robot_pose.t(1) << ", " << robot_pose.yaw() << "    dist=" << distance_to_goal << "     phi=" << delta_angle << std::endl;
////			// also abort if the robot does not move for a long time
////			{
////				boost::mutex::scoped_lock lock(odom_msg_mutex_);
////				robot_speed_x = odom_msg_.twist.twist.linear.x;
////				robot_speed_theta = odom_msg_.twist.twist.angular.z;
////			}
////			if (fabs(robot_speed_x) > 0.01 || fabs(robot_speed_theta) > 0.01)
////				last_robot_movement = ros::Time::now();
////			double robot_freeze_time = (ros::Time::now()-last_robot_movement).toSec();
////			//std::cout << "robot_freeze_time" << robot_freeze_time << std::endl;
////			boost::mutex::scoped_lock lock(nav_pilot_event_status_mutex_);
////			if (nav_pilot_event_status_.compare("PlanAndDrive")!=0 ||
////					(distance_to_goal<goal_position_tolerance*goal_position_tolerance && fabs(delta_angle)<goal_angle_tolerance) ||
////					(robot_freeze_time > robot_freeze_timeout))
////				break;
////			ros::spinOnce();
////		}
//	}
//
//	std::cout << "  Wall following successfully terminated." << std::endl;
//
//	// this sends the response back to the caller
//	PathActionServer::Result res;
//	wall_follow_action_server_->setSucceeded(res);
//#else
//	ROS_ERROR("ScitosDrive::wall_follow_callback: This function is not compiled. Install the MIRA Pilot addon and make sure it is found by cmake.");
//
//	// this sends the response back to the caller
//	PathActionServer::Result res;
//	wall_follow_action_server_->setAborted(res);
//#endif
//}


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
