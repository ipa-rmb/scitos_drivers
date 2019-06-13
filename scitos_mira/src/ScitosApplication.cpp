
#include "scitos_mira/ScitosApplication.h"
#include "scitos_mira/ScitosG5.h"

#include <cob_srvs/SetInt.h>
#include <iostream>

ScitosApplication::ScitosApplication() : ScitosModule(std::string ("Application")), status_service_name_("set_application_status_application_wet_cleaning")
{

}

void ScitosApplication::initialize()
{
	// offered services
	robot_->getMiraAuthority().publishService(*this);

	// offered channels
	application_status_sub_ = robot_->getRosNode().subscribe("/application_wet_cleaning_status", 1, &ScitosApplication::application_status_callback, this);
	application_status_channel_ = robot_->getMiraAuthority().publish<int>("AutomaticCleaningStatus");	// todo: hack: put to separate module

	// detections displays
	detections_channel_ = robot_->getMiraAuthority().publish<mira::maps::PointCloud2>("Detections"); // todo: hack: put to separate module
	dirt_detections_sub_ = robot_->getRosNode().subscribe("/dirt_detector_topic", 1, &ScitosApplication::publish_detections, this);
	trash_detections_sub_ = robot_->getRosNode().subscribe("/trash_detector_topic", 1, &ScitosApplication::publish_detections, this);
}

template <typename Reflector>
void ScitosApplication::reflect(Reflector& r)
{
	r.method("start_application", &ScitosApplication::startApplication, this, "This method starts the cleaning application.");
	r.method("start_application_without_cleaning", &ScitosApplication::startApplicationWithoutCleaning, this, "This method starts the cleaning application without using the cleaning device.");
	r.method("pause_application", &ScitosApplication::pauseApplication, this, "This method pauses the cleaning application.");
	r.method("stop_application", &ScitosApplication::stopApplication, this, "This method stops the cleaning application.");
}

int ScitosApplication::changeApplicationStatus(APPLICATION_STATUS status) const
{
	std::cout << ">> Changing application status to " << status << std::endl;
	cob_srvs::SetIntRequest request;
	cob_srvs::SetIntResponse response;
	request.data = status;
	ros::service::waitForService(status_service_name_);
	if (!ros::service::call(status_service_name_, request, response)) {
		std::cout << ">>>> Service call to " << status_service_name_ << " failed" << std::endl;
		return 1;
	}

	return 0;
}

int ScitosApplication::startApplication(void)
{
	const int result = changeApplicationStatus(APPLICATION_START);
	if (result == 0)
		robot_->getRosNode().setParam("use_cleaning_device", true);
	return result;
}


int ScitosApplication::startApplicationWithoutCleaning(void)
{
	const int result = changeApplicationStatus(APPLICATION_START);
	if (result == 0)
		robot_->getRosNode().setParam("use_cleaning_device", false);
	return result;
}


int ScitosApplication::stopApplication(void)
{
	return changeApplicationStatus(APPLICATION_STOP);
}


int ScitosApplication::pauseApplication(void)
{
	return changeApplicationStatus(APPLICATION_PAUSE);
}


void ScitosApplication::application_status_callback(const std_msgs::Int32::ConstPtr& msg)
{
	mira::ChannelWrite<int> w = application_status_channel_.write();
	w->timestamp = mira::Time::now(); // optional
	w->value() = msg->data; // oder 1
	w.finish();
}


void ScitosApplication::publish_detections(const cob_object_detection_msgs::DetectionArray::ConstPtr& object_detection_msg)
{
	mira::ChannelWrite<mira::maps::PointCloud2> w = detections_channel_.write();
	w->value().clear();
	w->timestamp = mira::Time::now(); // optional
	for (size_t k = 0; k < object_detection_msg->detections.size(); ++k)
	{
		const cob_object_detection_msgs::Detection& detection = object_detection_msg->detections[k];
		w->value().push_back(mira::Point2f(detection.pose.pose.position.x, detection.pose.pose.position.y));
	}
	w.finish();
}

std::ostream& operator<<(std::ostream &o,APPLICATION_STATUS status)
{
	switch(status)
	{
		case APPLICATION_START:
			return o << "STARTING APPLICATION";
		case APPLICATION_STOP:
			return o << "STOPPING APPLICATION";
		case APPLICATION_PAUSE:
			return o << "PAUSING APPLICATION";
		default:
			return o << "(invalid value for application_status)";
	}
}
