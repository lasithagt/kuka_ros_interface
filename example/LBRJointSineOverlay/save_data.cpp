
#include <iiwa_ros.h>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <iostream>
using namespace std;

ofstream myfile_JP_com;
ofstream myfile_JP_state;
ofstream myfile_JT;
// ofstream myfile_JV;
// ofstream myfile_CP;
// ofstream myfile_CW;
ofstream myfile_NETFT;

void callbackCommandJP(const iiwa_msgs::JointPosition::ConstPtr& msg) {

	myfile_JP_com << ros::Time::now().toSec() << ",";// (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * pow(10,-9) << ","; 
	myfile_JP_com << msg->position.a1 << ","; 
	myfile_JP_com << msg->position.a2 << ",";
	myfile_JP_com << msg->position.a3 << ",";
	myfile_JP_com << msg->position.a4 << ",";
	myfile_JP_com << msg->position.a5 << ",";
	myfile_JP_com << msg->position.a6 << ",";
	myfile_JP_com << msg->position.a7 << ",";
	myfile_JP_com << endl;
}

void callbackStateJP(const iiwa_msgs::JointPosition::ConstPtr& msg) {
	myfile_JP_state << ros::Time::now().toSec() << ",";//(double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * pow(10,-9) << ","; 
	myfile_JP_state << msg->position.a1 << ","; 
	myfile_JP_state << msg->position.a2 << ",";
	myfile_JP_state << msg->position.a3 << ",";
	myfile_JP_state << msg->position.a4 << ",";
	myfile_JP_state << msg->position.a5 << ",";
	myfile_JP_state << msg->position.a6 << ",";
	myfile_JP_state << msg->position.a7 << ",";
	myfile_JP_state << endl;
}

void callbackStateJT(const iiwa_msgs::JointTorque::ConstPtr& msg) {
	myfile_JT << ros::Time::now().toSec() << ",";//(double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * pow(10,-9) << ","; 
	myfile_JT << msg->torque.a1 << ","; 
	myfile_JT << msg->torque.a2 << ",";
	myfile_JT << msg->torque.a3 << ",";
	myfile_JT << msg->torque.a4 << ","; 
	myfile_JT << msg->torque.a5 << ",";
	myfile_JT << msg->torque.a6 << ",";
	myfile_JT << msg->torque.a7 << ",";
	myfile_JT << endl;
}

/*void callbackStateJV(const iiwa_msgs::JointVelocity::ConstPtr& msg) {
	myfile_JV << (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * pow(10,-9) << ","; 
	myfile_JV << msg->velocity.a1 << ","; 
	myfile_JV << msg->velocity.a2 << ",";
	myfile_JV << msg->velocity.a3 << ",";
	myfile_JV << msg->velocity.a4 << ",";
	myfile_JV << msg->velocity.a5 << ",";
	myfile_JV << msg->velocity.a6 << ",";
	myfile_JV << msg->velocity.a7 << ",";
	myfile_JV << endl;
}*/

/*void callbackCP(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	myfile_CP << (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * pow(10,-9) << ","; 
	myfile_CP << msg->pose.position.x << ","; 
	myfile_CP << msg->pose.position.y << ",";
	myfile_CP << msg->pose.position.z << ",";
	myfile_CP << msg->pose.orientation.x << ",";
	myfile_CP << msg->pose.orientation.y << ",";
	myfile_CP << msg->pose.orientation.z << ",";
	myfile_CP << msg->pose.orientation.w << ",";
	myfile_CP << endl;
}*/

/*void callbackCW(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	myfile_CW << (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * pow(10,-9) << ","; 
	myfile_CW << msg->wrench.force.x << ","; 
	myfile_CW << msg->wrench.force.y << ",";
	myfile_CW << msg->wrench.force.z << ",";
	myfile_CW << endl;	
}*/

void callbackNETFT(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	myfile_NETFT << ros::Time::now().toSec() << ",";//(double)msg->header.stamp.sec + (double)msg->header.stamp.nsec * pow(10,-9) << ","; 
	myfile_NETFT << msg->wrench.force.x << ","; 
	myfile_NETFT << msg->wrench.force.y << ",";
	myfile_NETFT << msg->wrench.force.z << ",";
	myfile_NETFT << endl;
}


int main (int argc, char **argv) {

	ros::init(argc, argv, "SaveData");
	ros::NodeHandle nh("~");

  	myfile_JP_state.open("/home/lasitha/Documents/ROS_ws/kuka_ws/src/kuka_torque/kuka_data/JP_state.txt");
  	myfile_JP_com.open("/home/lasitha/Documents/ROS_ws/kuka_ws/src/kuka_torque/kuka_data/JP_com.txt");
  	myfile_JT.open("/home/lasitha/Documents/ROS_ws/kuka_ws/src/kuka_torque/kuka_data/JT.txt");
  	//myfile_JV.open("/home/lasitha/Documents/ROS_ws/kuka_ws/src/kuka-iiwa/iiwa_stack_examples/iiwa_tool_examples/src/Kuka_data/JV.txt");
  	//myfile_CP.open("/home/lasitha/Documents/ROS_ws/kuka_ws/src/kuka-iiwa/iiwa_stack_examples/iiwa_tool_examples/src/Kuka_data/CP.txt");
  	//myfile_CW.open("/home/lasitha/Documents/ROS_ws/kuka_ws/src/kuka-iiwa/iiwa_stack_examples/iiwa_tool_examples/src/Kuka_data/CW.txt");
  	myfile_NETFT.open("/home/lasitha/Documents/ROS_ws/kuka_ws/src/kuka_torque/kuka_data/NETFT.txt");

  	cout << "Running...." << endl;
  	myfile_JP_com << std::setprecision(14);
  	myfile_JP_state << std::setprecision(14);
  	myfile_JT << std::setprecision(14);
  	//myfile_JV << std::setprecision(14);
  	//myfile_CW << std::setprecision(14);
  	//myfile_CP << std::setprecision(14);
  	myfile_NETFT << std::setprecision(14);

	ros::Subscriber subJPC   = nh.subscribe("/kuka/KUKAJointPositionCommand", 1, callbackCommandJP);
	ros::Subscriber subJPS   = nh.subscribe("/kuka/KUKAJointPosition", 1, callbackStateJP);
	ros::Subscriber subJT    = nh.subscribe("/kuka/KUKAActualTorque", 1, callbackStateJT);
	//ros::Subscriber subJV    = nh.subscribe("/iiwa/state/JointVelocity", 1, callbackStateJV);
	//ros::Subscriber subCP    = nh.subscribe("/iiwa/re/state/CartesianPose", 1, callbackCP);
	//ros::Subscriber subCW    = nh.subscribe("/iiwa/state/CartesianWrench", 1, callbackCW);
	ros::Subscriber subNETFT = nh.subscribe("/netft_data", 1, callbackNETFT);


	ros::AsyncSpinner spinner(4);
	spinner.start();

	while(ros::ok()) {

	}

	myfile_JP_state.close();
	myfile_JP_com.close();
	myfile_JT.close();
	//myfile_JV.close();
	//myfile_CP.close();
	//myfile_CW.close();
	myfile_NETFT.close();
	return 0;
}