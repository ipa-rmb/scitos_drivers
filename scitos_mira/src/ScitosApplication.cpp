
#include "scitos_mira/ScitosApplication.h"
#include "scitos_mira/ScitosG5.h"

#include <cob_srvs/SetInt.h>

ScitosApplication::ScitosApplication() : ScitosModule(std::string ("Application"))
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

// todo: later we should pass a parameter for the service_name of the respective application that shall be started
int ScitosApplication::startApplication(void)
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


int ScitosApplication::startApplicationWithoutCleaning(void)
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


int ScitosApplication::stopApplication(void)
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


int ScitosApplication::pauseApplication(void)
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
	std::cout << "detections size " << object_detection_msg->detections.size() << std::endl;
	for (size_t k = 0; k < object_detection_msg->detections.size(); ++k)
	{
		const cob_object_detection_msgs::Detection& detection = object_detection_msg->detections[k];
		std::cout << "detection on (" << detection.pose.pose.position.x << ", " << detection.pose.pose.position.y << ")" << std::endl;
		w->value().push_back(mira::Point2f(detection.pose.pose.position.x, detection.pose.pose.position.y));
	}
	w.finish();
}
