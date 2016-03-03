#ifndef PR2_MOTION_H
#define PR2_MOTION_H




#include <ros/ros.h>
#include <pr2motion/Arm_Right_MoveAction.h>
#include <pr2motion/Head_MoveAction.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/Gripper_OperateAction.h>
#include <pr2motion/Gripper_Left_Stop.h>
#include <pr2motion/Gripper_Right_Stop.h>
#include <pr2motion/connect_port.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction> ArmClient;
typedef actionlib::SimpleActionClient<pr2motion::Head_MoveAction> HeadClient;
typedef actionlib::SimpleActionClient<pr2motion::Gripper_OperateAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2motion::InitAction> InitClient;


class Pr2MotionBridge {
public:
	Pr2MotionBridge(ros::NodeHandle node_handle_);
	void moveArmGtp(string arm);
	void lookAt(string frame,bool follow);
	bool openGripper(string gripper);
	bool closeGripper(string gripper);
	bool releaseGripper(string gripper);
	bool grabGripper(string gripper);
	void stopGripper(string gripper);
	void stopHead();
	void stopArm();

	bool isArmCompleted();
	bool hasArmSucceded();
private:
	ros::NodeHandle node_handle_;

	ArmClient arm_client_;
	HeadClient head_client_;
	GripperClient gripper_client_;
	InitClient init_client_;
	
	ros::ServiceClient gripper_left_stop_client_;
	ros::ServiceClient gripper_right_stop_client_;
	ros::ServiceClient connect_port_client_;

	bool is_arm_completed_;
	bool has_arm_succeded_;
	boost::mutex mutex_is_arm_completed_;


	void setArmCompleted(bool completed);
	void setArmSucceded(bool succeded);

	void armDoneCb(const actionlib::SimpleClientGoalState& state,
	            const pr2motion::Arm_Right_MoveResultConstPtr& result);


	int getSide(string side);
	bool operateGripper(int gripper, int mode);
    void connect(string local, string remote);

};


#endif