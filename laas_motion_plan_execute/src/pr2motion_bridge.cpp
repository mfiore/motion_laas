#include <laas_motion_plan_execute/pr2motion_bridge.h>

Pr2MotionBridge::Pr2MotionBridge(ros::NodeHandle node_handle):
node_handle_(node_handle),
arm_client_("pr2motion/Arm_Right_Move",true),
head_client_("pr2motion/Head_Move",true),
gripper_client_("pr2motion/Gripper_Operate",true),
init_client_("pr2motion/Init",true)

{
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE initializing pr2motion");
	init_client_.waitForServer();
	pr2motion::InitGoal goal_msg;
	init_client_.sendGoal(goal_msg);
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE waiting for pr2motion actions and services");
	arm_client_.waitForServer();
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE connected to arm client");
	head_client_.waitForServer();
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE connected to head client");
	gripper_client_.waitForServer();
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE connected to gripper client");
	gripper_left_stop_client_=node_handle_.serviceClient<pr2motion::Gripper_Left_Stop>("pr2motion/Gripper_Left_Stop");
	gripper_right_stop_client_=node_handle_.serviceClient<pr2motion::Gripper_Right_Stop>("pr2motion/Gripper_Right_Stop");
	gripper_left_stop_client_.waitForExistence();
	gripper_right_stop_client_.waitForExistence();
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE connected to gripper services");
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE connecting to gtp and joint states");
	connect_port_client_=node_handle_.serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
	connect_port_client_.waitForExistence();
	connect("joint_state","joint_states");
	connect("head_controller_state","/head_traj_controller/state");
	connect("traj","gtp_trajectory");
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE connected to pr2 motion");	
}

void Pr2MotionBridge::connect(string local, string remote) {
	pr2motion::connect_port srv;
	srv.request.local=local;
	srv.request.remote=remote;
	connect_port_client_.call(srv);

}

void Pr2MotionBridge::moveArmGtp(string arm) {
	has_arm_succeded_=false;
	is_arm_completed_=false;

	pr2motion::Arm_Right_MoveGoal arm_goal;
	//arm_goal.side.value=getSide(arm);
	arm_goal.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
	arm_goal.path_mode.value=pr2motion::pr2motion_PATH_MODE::pr2motion_PATH_PORT;

	arm_client_.sendGoal(arm_goal,boost::bind(&Pr2MotionBridge::armDoneCb,this,_1,_2),
		ArmClient::SimpleActiveCallback(),
		ArmClient::SimpleFeedbackCallback());
}
void Pr2MotionBridge::armDoneCb(const actionlib::SimpleClientGoalState& state,
            const pr2motion::Arm_Right_MoveResultConstPtr& result) {
	ROS_INFO("LAAS_MOTION_PLAN_EXECUTE arm motion completed with result %d %s",result->genom_success,
		result->genom_exdetail.c_str());
	if (result->genom_success) {
		setArmSucceded(true);
	}
	else {
		setArmSucceded(false);
	}
	setArmCompleted(true);
}

void Pr2MotionBridge::stopArm() {
	arm_client_.cancelGoal();
	setArmSucceded(false);
	setArmCompleted(true);
}
void Pr2MotionBridge::stopGripper(string gripper) {
	gripper_client_.cancelGoal();
}

void Pr2MotionBridge::lookAt(string frame, bool follow) {
	pr2motion::Head_MoveGoal head_goal;
	head_goal.head_mode.value=pr2motion::pr2motion_HEAD_MODE::pr2motion_HEAD_LOOKAT;
	head_goal.head_target_frame=frame;
	head_client_.sendGoal(head_goal);
}
void Pr2MotionBridge::stopHead() {
	head_client_.cancelGoal();
}

bool Pr2MotionBridge::operateGripper(int gripper, int mode) {
	pr2motion::Gripper_OperateGoal gripper_goal;
	gripper_goal.side.value=gripper;
	gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
	actionlib::SimpleClientGoalState state=gripper_client_.sendGoalAndWait(gripper_goal,ros::Duration(10.0));
	if (state!=actionlib::SimpleClientGoalState::SUCCEEDED) 
		return false;
	else return true;
} 

bool Pr2MotionBridge::openGripper(string gripper) {
	return operateGripper(getSide(gripper),pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN);
}

bool Pr2MotionBridge::closeGripper(string gripper) {
	return operateGripper(getSide(gripper),pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_CLOSE);
}

bool Pr2MotionBridge::releaseGripper(string gripper) {
	return operateGripper(getSide(gripper),pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_RELEASE);
}
bool Pr2MotionBridge::grabGripper(string gripper) {
	return operateGripper(getSide(gripper),pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_GRAB);
}


int Pr2MotionBridge::getSide(string side) {

	if (side=="RIGHT") {
		return pr2motion::pr2motion_SIDE::pr2motion_RIGHT;
	}
	else {
		return pr2motion::pr2motion_SIDE::pr2motion_LEFT;		
	}
}

bool Pr2MotionBridge::isArmCompleted() {
	boost::lock_guard<boost::mutex> lock(mutex_is_arm_completed_);
	return is_arm_completed_;
}
void Pr2MotionBridge::setArmCompleted(bool completed) {
	boost::lock_guard<boost::mutex> lock(mutex_is_arm_completed_);
	is_arm_completed_=true;
}

bool Pr2MotionBridge::hasArmSucceded() {
	return has_arm_succeded_;
}

void Pr2MotionBridge::setArmSucceded(bool succeded) {
	has_arm_succeded_=succeded;
}
