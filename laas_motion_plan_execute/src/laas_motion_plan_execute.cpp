#include <ros/ros.h>


#include <actionlib/server/simple_action_server.h>

#include <action_management_msgs/ManageActionAction.h>
#include <motion_msgs/MotionPlanExecuteAction.h>

#include <laas_motion_plan_execute/gtp_bridge.h>
#include <laas_motion_plan_execute/pr2motion_bridge.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <std_srvs/Empty.h>


#include <vector>
#include <string>

typedef actionlib::SimpleActionServer<motion_msgs::MotionPlanExecuteAction> Server;

using namespace std;

string robot_name_;
GtpBridge *gtp_bridge_;
Pr2MotionBridge *pr2motion_bridge_;

vector<string> gtp_actions_;

bool is_paused_;
boost::mutex mutex_is_paused_;



map<string,string> getParametersMap(vector<common_msgs::Parameter> parameters) {
	map<string,string> result;
	for (int i=0; i<parameters.size();i++) {
		result[parameters[i].name]=parameters[i].value;
	}
	return result;
}


string getGtpSide(int side) {
	if (side==0) {
		return "RIGHT";
	}
	else {
		return "LEFT";
	}
}

bool isPaused() {
	boost::lock_guard<boost::mutex> lock(mutex_is_paused_);
	return is_paused_;
}
void setIsPaused(bool is_paused) {
	boost::lock_guard<boost::mutex> lock(mutex_is_paused_);
	is_paused_=is_paused;
}


bool pauseMotion(std_srvs::Empty::Request &req,
             std_srvs::Empty::Response &res) {
	setIsPaused(true);
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE motion paused");
	return true;
}

bool resumeMotion(std_srvs::Empty::Request &req,
             std_srvs::Empty::Response &res) {
	setIsPaused(false);
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE motion resumed");
	return true;
}



motion_msgs::MotionPlanExecuteResult executeGtpPlan(Server* server, string goal, map<string,string> goal_parameters) {
	motion_msgs::MotionPlanExecuteFeedback feedback;
	motion_msgs::MotionPlanExecuteResult result;

	ros::Rate r(3);

	//if this is an action supported by gtp create a gtp plan
	gtp_ros_msg::ReqAns gtp_report=gtp_bridge_->planGtpTask(goal,goal_parameters);
	if (gtp_report.success) { //if the plan is successfull
		for (int i=0; i<gtp_report.subTrajs.size();i++) {  //for each trajectory of the plan
			gtp_ros_msg::SubTraj sub_traj=gtp_report.subTrajs[i];
			string arm=getGtpSide(sub_traj.armId); //convert gtp arm to RIGHT or LEFT
			bool motion_ok=true;
			
			if (sub_traj.subTrajName=="release") {
				motion_ok=pr2motion_bridge_->openGripper(arm);
			}
			else if (sub_traj.subTrajName=="grasp") {
				motion_ok=pr2motion_bridge_->closeGripper(arm);
			}
			else if (sub_traj.subTrajName=="navigate") {
				ROS_WARN("LAAS_MOTION_PLAN_EXECUTE navigate for gtp bridge is still not implemented");
			}
			else { //arm traj
					bool ok=gtp_bridge_->loadGtpTrajectory(i); //we load it
					if (ok) { 
						pr2motion_bridge_->moveArmGtp(arm); 
						//we wait until the movement is completed or there is some problem
						while (!pr2motion_bridge_->isArmCompleted() && ros::ok() && !server->isPreemptRequested() &&
							!isPaused()) {
							r.sleep();
						}
						if (!ros::ok()) return result;
						if (server->isPreemptRequested()) {
							ROS_INFO("LAAS_MOTION_PLAN_EXECUTE preempted");
							pr2motion_bridge_->stopArm();
							result.report.status="PREEMPTED";
							result.report.details="";
							return result;
						}
						if (isPaused()) {
							result.report.status="PAUSED";
							return result;
						}
						if (pr2motion_bridge_->isArmCompleted() && !pr2motion_bridge_->hasArmSucceded()) {
							motion_ok=false; //we just set the flag, which will be read with the result. 
						}
					}
					else {
						ROS_INFO("LAAS_MOTION_PLAN_EXECUTE failed to load gtp trajectory");
						result.report.status="FAILED";
						result.report.details="failed to laod gtp trajectory";
						return result;
					}
			}
			//Now we either publish a feedback if the trajectory was correctly executed or we quit.
			if (motion_ok) { 
				feedback.report.status="RUNNING";
				feedback.report.details="trajectory "+boost::lexical_cast<string>(i+1)+" of "+boost::lexical_cast<string>(gtp_report.subTrajs.size())+" completed";
				server->publishFeedback(feedback);
			}
			else {
				ROS_INFO("LAAS_MOTION_PLAN_EXECUTE arm action failed");
				result.report.status="FAILED";
				result.report.details="motion failed";
				return result;
			}
		}
	}
	result.report.status="COMPLETED";
	result.report.details="";
	return result;	
}

//feedback
void planExecute(const motion_msgs::MotionPlanExecuteGoalConstPtr &goal,Server* server) {
	motion_msgs::MotionPlanExecuteResult result;

	map<string,string> goal_parameters=getParametersMap(goal->parameters); //convert the parameters to a map
	string goal_name=goal->goal;

	ros::Rate r(3);
	while (isPaused()) {
		r.sleep();
	}
	if (std::find(gtp_actions_.begin(),gtp_actions_.end(),goal->goal)!=gtp_actions_.end()) { 
		bool completed=false;
		while (ros::ok()) { //the ifs in the loop will return when they have a result, but if the plan is paused the loop will go on
		motion_msgs::MotionPlanExecuteResult gtp_status=executeGtpPlan(server,goal_name,goal_parameters);
		
			if (gtp_status.report.status!="FAILED") {
				server->setAborted(gtp_status);
				return;
			}
			else if (gtp_status.report.status=="COMPLETED") {
				server->setSucceeded(gtp_status);
				return;
			}
			else if (gtp_status.report.status=="PREEMPTED") {
				server->setPreempted(gtp_status);
				return;
			}
		}
	}
	else {
		bool ok;
		if (goal_name=="open_gripper") {
			ok=pr2motion_bridge_->openGripper(goal_parameters["arm"]);
		}
		else if (goal_name=="close_gripper") {
			ok=pr2motion_bridge_->closeGripper(goal_parameters["arm"]);
		}	
		if (ok) {
			result.report.status="COMPLETED";
			result.report.details="";
			server->setSucceeded(result);
		}	
		else  {
			result.report.status="FAILED";
			result.report.details="";
			server->setAborted(result);
		}
	}

}

int main(int argc, char **argv) {
	ros::init(argc,argv,"motion_plan_execute");
	ros::NodeHandle node_handle;

	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE started node");
	node_handle.getParam("/robot/name",robot_name_);
	node_handle.getParam("/supervision/gtp_actions",gtp_actions_);

	setIsPaused(false);

	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE robot name is %s",robot_name_.c_str());
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE gtp actions are:");
	for (int i=0; i<gtp_actions_.size();i++) {
		ROS_INFO("- %s",gtp_actions_[i].c_str());
	}

	gtp_bridge_=new GtpBridge(node_handle,robot_name_);
	pr2motion_bridge_=new Pr2MotionBridge(node_handle);

	ros::ServiceServer pause_motion=node_handle.advertiseService("/motion/pause_motion",pauseMotion);
	ros::ServiceServer resume_motion=node_handle.advertiseService("/motion/resume_motion",resumeMotion);

	Server server(node_handle,"motion/motion_plan_execute/", boost::bind(&planExecute,_1,&server),false); 
	server.start();
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE ready");
	ros::spin();
}
