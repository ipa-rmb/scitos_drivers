/* ScitosApplication.h
 *
 * Module for controlling the application between ROS and Mira (start, pause...)
 *
 */

#ifndef SCITOSAPPLICATION_H
#define SCITOSAPPLICATION_H

#include <ros/ros.h>
#include <scitos_mira/ScitosModule.h>
#include <maps/PointCloudTypes.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <std_msgs/Int32.h>

class ScitosApplication: public ScitosModule {

public:
	static ScitosModule* Create() {
		return new ScitosApplication();
	}

	void initialize();

	template <typename Reflector> void reflect(Reflector& r);
	int startApplication(void);
	int startApplicationWithoutCleaning(void);
	int pauseApplication(void);
	int stopApplication(void);
	void application_status_callback(const std_msgs::Int32::ConstPtr& msg);
	void publish_detections(const cob_object_detection_msgs::DetectionArray::ConstPtr& object_detection_msg);


private:
	ScitosApplication();

	mira::Channel<mira::maps::PointCloud2> detections_channel_; // todo: hack: put to separate module
	mira::Channel<int> application_status_channel_;		// todo: hack: put to separate module
	ros::Subscriber application_status_sub_;		// todo: hack: put to separate module
	ros::Subscriber dirt_detections_sub_;
	ros::Subscriber trash_detections_sub_;
};

#endif /* SCITOSAPPLICATION_H */
