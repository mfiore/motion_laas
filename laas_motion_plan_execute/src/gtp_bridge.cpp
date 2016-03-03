#include <laas_motion_plan_execute/gtp_bridge.h>

GtpBridge::GtpBridge(ros::NodeHandle node_handle,string robot_name):
node_handle_(node_handle),
gtp_client_("gtp_ros_server",true),
robot_name_(robot_name)
{
	gtp_client_.waitForServer();
	gtp_id_=2;
}



gtp_ros_msg::ReqAns GtpBridge::planGtpTask(string action, map<string,string> parameters) {
	gtp_ros_msg::requestGoal gtp_goal;
	gtp_ros_msg::Req gtp_request;


	gtp_request.requestType="planning";
	gtp_request.actionName=action;
	gtp_ros_msg::Ag main_agent;
	main_agent.actionKey="mainAgent";
	main_agent.agentName=robot_name_;
	gtp_request.involvedAgents.push_back(main_agent);
	vector<gtp_ros_msg::Obj> gtp_objects;

	if (parameters.find("target_agent")!=parameters.end()) {
		gtp_ros_msg::Ag target_agent;
		if (action!="navigateTo") {
			target_agent.actionKey="targetAgent";
		}
		else {
			target_agent.actionKey="target";
		}
		target_agent.agentName=parameters["target_agent"];
		gtp_request.involvedAgents.push_back(target_agent);
	}
	if (parameters.find("target_object")!=parameters.end()) {
		gtp_ros_msg::Obj obj;
		obj.actionKey="target";
		obj.objectName=parameters["target_object"];
		
		gtp_request.involvedObjects.push_back(obj);
	}

	if (parameters.find("main_object")!=parameters.end()) {
		gtp_ros_msg::Obj obj;
		obj.actionKey="mainObject";
		obj.objectName=parameters["main_object"];
	
		gtp_request.involvedObjects.push_back(obj);
	}

	if (parameters.find("support_object")!=parameters.end()) {
		gtp_ros_msg::Obj obj;
		obj.actionKey="supportObject";
		obj.objectName=parameters["support_object"];

		gtp_request.involvedObjects.push_back(obj);
	}

	if (parameters.find("pose_name")!=parameters.end()) {
		gtp_ros_msg::Data data;
		data.dataKey="confName";
		data.dataValue=parameters["pose_name"];

		gtp_request.data.push_back(data);
	}
	if (parameters.find("hand")!=parameters.end()) {
		gtp_ros_msg::Data data;
		data.dataKey="hand";
		data.dataValue=parameters["hand"];

		gtp_request.data.push_back(data);
	}



	gtp_request.predecessorId.actionId=gtp_id_;
	gtp_request.predecessorId.alternativeId=-1;

	gtp_goal.req=gtp_request;

	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE sending goal to gtp with parameters:");
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE request type %s",gtp_request.requestType.c_str());
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE action name type %s",gtp_request.actionName.c_str());
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE involved agents");
	for (int i=0; i<gtp_request.involvedAgents.size();i++) {
		ROS_INFO("LAAS_MOTION_PLAN_EXECUTE %s %s",gtp_request.involvedAgents[i].actionKey.c_str(),gtp_request.involvedAgents[i].agentName.c_str());
	}

	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE sending goal");
	gtp_client_.sendGoal(gtp_goal);
	gtp_client_.waitForResult();
	gtp_ros_msg::requestResultConstPtr result=gtp_client_.getResult();

	if (result->ans.success) {
		gtp_id_=result->ans.identifier.actionId;
	}
	else {
	
		gtp_id_=-1;
	}
	return result->ans;
}

gtp_ros_msg::ReqAns GtpBridge::getDetails() {
	gtp_ros_msg::requestGoal gtp_goal;
	gtp_ros_msg::Req gtp_request;

	gtp_request.requestType="details";
	gtp_request.loadAction.actionId=gtp_id_;
	gtp_request.loadAction.alternativeId=0;

	gtp_goal.req=gtp_request;

	gtp_client_.sendGoal(gtp_goal);
	gtp_client_.waitForResult();


	gtp_ros_msg::requestResultConstPtr result=gtp_client_.getResult();

	return result->ans;
}

void GtpBridge::removeAttachement() {
	gtp_ros_msg::requestGoal gtp_goal;
	gtp_ros_msg::Req gtp_request;

	gtp_request.requestType="RemoveGtpAttachements";

	gtp_goal.req=gtp_request;

	gtp_client_.sendGoal(gtp_goal);


}

void GtpBridge::update() {
	gtp_ros_msg::requestGoal gtp_goal;
	gtp_ros_msg::Req gtp_request;

	gtp_request.requestType="update";

	gtp_goal.req=gtp_request;

	gtp_client_.sendGoal(gtp_goal);
	return;
}

bool GtpBridge::loadGtpTrajectory(int i) {
		gtp_ros_msg::requestGoal gtp_goal;
		gtp_ros_msg::Req gtp_request;

		gtp_request.requestType="load";

		gtp_request.loadAction.actionId=gtp_id_;
		gtp_request.loadAction.alternativeId=0;
		gtp_request.loadSubTraj=i;

		gtp_goal.req=gtp_request;

		gtp_client_.sendGoal(gtp_goal);
		gtp_client_.waitForResult();
		gtp_ros_msg::requestResultConstPtr result=gtp_client_.getResult();


		return result->ans.success;
}