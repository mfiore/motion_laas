#ifndef GTP_BRIDGE_H
#define GTP_BRIDGE_H

#include <ros/ros.h>
#include <gtp_ros_msg/requestAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>
using namespace std;

typedef actionlib::SimpleActionClient<gtp_ros_msg::requestAction> GtpClient;


class GtpBridge {
public:
	GtpBridge(ros::NodeHandle node_handle, string robot_name);
	gtp_ros_msg::ReqAns planGtpTask(string action, map<string,string> parameters);
	void removeAttachement();
	bool loadGtpTrajectory(int i);
	void update();
	gtp_ros_msg::ReqAns getDetails();



private:
	map<string,string> convertParameters(map<string,string> parameters);

	string robot_name_;
	int gtp_id_;
	GtpClient gtp_client_;
	ros::NodeHandle node_handle_;
};

#endif 