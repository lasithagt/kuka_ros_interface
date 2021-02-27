// version: 1.7
/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," 
without warranty of any kind, including without limitation the warranties 
of merchantability, fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable 
for any particular purpose. In no event shall KUKA be responsible for loss 
or damages arising from the installation or use of the Software, 
including but not limited to any indirect, punitive, special, incidental 
or consequential damages of any character including, without limitation, 
damages for loss of goodwill, work stoppage, computer failure or malfunction, 
or any and all other commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by KUKA. 
Should the Software prove defective, KUKA is not liable for the entire cost 
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2015 
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned 
to KUKA Roboter GmbH immediately upon request.  
This material and the information illustrated or contained herein may not be used, 
reproduced, stored in a retrieval system, or transmitted in whole 
or in part in any way - electronic, mechanical, photocopying, recording, 
or otherwise, without the prior written consent of KUKA Roboter GmbH.  

*/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "LBRWrenchSineOverlayClient.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

//******************************************************************************
LBRWrenchSineOverlayClient::LBRWrenchSineOverlayClient(ros::NodeHandle &nh) 
   : command_active(false)
   ,is_valid(false)
   ,is_valid_wrench(false)
   ,is_command_wrench(false)
   ,sample_time(0.0)
   ,is_command(false)
   ,joint_cmd_active(false)
{
  printf("KUKA LBR initilized ...\n");

  // Subscribers
  sub_command_wrench = nh.subscribe("/kuka/command/command_wrench", 1, &LBRWrenchSineOverlayClient::getKUKAWrenchCmd, this);
  sub_command_joint  = nh.subscribe("/kuka/command/command_position", 1, &LBRWrenchSineOverlayClient::getKUKAJointCmd, this);
  sub_command_active = nh.subscribe("/kuka/command/active", 1, &LBRWrenchSineOverlayClient::is_command_active, this);

  // Publishers
  pub_torque       = nh.advertise<iiwa_msgs::JointTorque>("/kuka/state/KUKAActualTorque", 1); 
  pub_ext_torque   = nh.advertise<iiwa_msgs::JointTorque>("/kuka/state/KUKAExtTorque", 1); 
  pub_position     = nh.advertise<iiwa_msgs::JointPosition>("/kuka/state/KUKAJointPosition", 1); 
  pub_position_com = nh.advertise<iiwa_msgs::JointPosition>("/kuka/state/KUKAJointPositionCommand", 1); 
  joint_vel_pub_   = nh.advertise<iiwa_msgs::JointVelocity>("/kuka/state/KUKAJointVelocity", 1);   
  pub_kuka_time    = nh.advertise<std_msgs::Time>("/kuka/state/KUKATime", 1);
  
  // Set current time to zero
  
  kukaTorque.torque.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaExtTorque.torque.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaPosition.position.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaPositionCommanded.position.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaVelocity.velocity.quantity.resize(LBRState::NUMBER_OF_JOINTS);


  // initialize the toruque
  for(int i = 0; i< WRENCH_STATE; i++){ _wrench[i] = 0.0;}

  // compute the velocity
  curr_velocity.resize(7);
  filter_weights.col(0) << 0.25/0.001, 0.25/0.001, 0.25/0.001, 0.25/0.001;

  // Number of points to the moving average
  n_position_history = 5;

  for (int i=0; i < n_position_history; i++) {
      temp_joint_pos.col(i) << 0,-1.21026  ,0,-1.67354, 0,-0.463276, 0;    
  }

  temp_joint_vel = Eigen::MatrixXd::Zero(7, n_position_history-1);

  // Initialize filters.
  const double cutoff_hz = 40;
  lp_filter = new DiscreteTimeLowPassFilter<double>(cutoff_hz, sample_time);
}

//******************************************************************************
LBRWrenchSineOverlayClient::~LBRWrenchSineOverlayClient()
{
}
      
//******************************************************************************
void LBRWrenchSineOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   switch (newState)
   {
      // (re)initialize sine parameters when entering Monitoring
      case MONITORING_READY:
      {
         for(int i = 0; i<WRENCH_STATE; i++){_wrench[i] = 0.0;}
         break;
      }
      default:
      {
         break;
      }
   }
}

//******************************************************************************
void LBRWrenchSineOverlayClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
   // the base method.
   LBRClient::waitForCommand();
   
   // If we want to command wrenches, we have to command them all the time. Even in
   // waitForCommand(). This has to be done due to consistency checks. In this state 
   // it is only necessary, that some wrench values are sent.
   // The LBR does not take the specific value into account.
   if (robotState().getClientCommandMode() == WRENCH)
   {
      robotCommand().setWrench(_wrench);
   }
}

//******************************************************************************
void LBRWrenchSineOverlayClient::command()
{
  // In command(), the joint values have to be sent. Which is done by calling
  // the base method.
  mtx.lock();

  double curr_joint_pos[LBRState::NUMBER_OF_JOINTS];
  double joint_pos_desired[LBRState::NUMBER_OF_JOINTS];
  double curr_velocity[LBRState::NUMBER_OF_JOINTS];


  // Get the current joint positions
  memcpy(curr_joint_pos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  // Measured torque = Commanded torque + External Torque;
  double joint_measure_torque[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_measure_torque, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  // Get the commanded torque
  double joint_commanded[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_commanded, robotState().getCommandedTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  // Get the external toruqe on joints
  double joint_ext_torque[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_ext_torque, robotState().getExternalTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  curr_time.data.sec  = (robotState().getTimestampSec());
  curr_time.data.nsec = (robotState().getTimestampNanoSec());
  
  // compute the velocity
  computeVelocity(curr_joint_pos, curr_velocity);

  // publisht the states 
  publishState(joint_ext_torque, joint_measure_torque, curr_joint_pos, joint_commanded, curr_velocity, curr_time);

  // Check for corre
  robotCommand().setJointPosition(curr_joint_pos);
  if (robotState().getClientCommandMode() == WRENCH) { 
     
 
    for (int i=0; i<WRENCH_STATE; i++) {
        // _torques[i]       = 10*(-joint_pos_desired[i] + curr_joint_pos[i]); // offset;
      
    }

    // Set superposed joint torques
    if (joint_cmd_active) {
      // ROS_INFO_STREAM(_joints[6]);
      robotCommand().setJointPosition(_joints);
    } else {
      // robotCommand().setJointPosition(curr_joint_pos);
    }
    // ROS_INFO_STREAM(_wrench[2]);
    robotCommand().setWrench(_wrench);
  }
  
  mtx.unlock();
}

void LBRWrenchSineOverlayClient::publishState(double jointExtTorque[], double jointTorque[], double jointState[], double jointStateCommanded[], double jointVelocity[], std_msgs::Time kuka_time)
{

  memcpy(kukaExtTorque.torque.quantity.data(), jointExtTorque, 7*sizeof(double));
  kukaExtTorque.header.stamp = ros::Time::now();

  memcpy(kukaTorque.torque.quantity.data(), jointTorque, 7*sizeof(double));
  kukaTorque.header.stamp = ros::Time::now();

  memcpy(kukaPosition.position.quantity.data(), jointState, 7*sizeof(double));
  kukaPosition.header.stamp = ros::Time::now();


  memcpy(kukaPositionCommanded.position.quantity.data(), jointStateCommanded, 7*sizeof(double));
  kukaPositionCommanded.header.stamp = ros::Time::now();

  memcpy(kukaVelocity.velocity.quantity.data(), jointVelocity, 7*sizeof(double));
  kukaVelocity.header.stamp = ros::Time::now();

  curr_time.data.sec = kuka_time.data.sec;
  curr_time.data.nsec = kuka_time.data.nsec;

  pub_torque.publish(kukaTorque);
  pub_ext_torque.publish(kukaExtTorque);
  pub_position.publish(kukaPosition);
  // pub_position_Ipo.publish(kukaPositionIpo);
  pub_position_com.publish(kukaPositionCommanded);
  joint_vel_pub_.publish(kukaVelocity);
  pub_kuka_time.publish(curr_time);

}

// This function is to compute the joint position input to the KUKA arm.
void LBRWrenchSineOverlayClient::getKUKAJointCmd(const ros::MessageEvent<iiwa_msgs::JointPosition const>& event) {

  is_command = true;
  is_valid = true;
  joint_cmd_active = true;

  double time_diff = abs(event.getReceiptTime().toSec() - ros::Time::now().toSec());
  const iiwa_msgs::JointPosition::ConstPtr& msg = event.getMessage();

  // memcpy(_joints, msg->position.quantity.data(),  7*sizeof(double));

  // for (int i = 0;i < LBRState::NUMBER_OF_JOINTS; i++) {
  //   if (abs(kukaPosition.position.quantity[i] - msg->position.quantity[i]) > 0.2) {
  //     is_valid = false;
  //   }
  // }

  if (is_valid==true && command_active==true && time_diff<0.1) {
    memcpy(_joints, msg->position.quantity.data(), 7*sizeof(double));
    
  } else {
    memcpy(_joints, kukaPosition.position.quantity.data(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    is_valid = false;
  }
  


}


// This function is to compute the torque input to the KUKA arm.
void LBRWrenchSineOverlayClient::getKUKAWrenchCmd(const ros::MessageEvent<iiwa_msgs::Wrench const>& event) {

  is_command_wrench = true;
  is_valid_wrench = true;

  double time_diff = abs(event.getReceiptTime().toSec() - ros::Time::now().toSec());
  const iiwa_msgs::Wrench::ConstPtr& msg = event.getMessage();


  // memcpy(_wrench, msg->wrench.quantity.data(),  6*sizeof(double));

  // for (int i = 0;i < WRENCH_STATE; i++) {
   // if (abs(kukaPosition.position.quantity[i] - msg->position.quantity[i]) > 0.2) {
   // is_valid = false;
   // }
  // }

  if (is_valid_wrench==true && command_active==true && time_diff<0.1) {
   // memcpy(_wrench, msg->wrench.quantity.data(), 6*sizeof(double));
    
  } else {

   for(int i = 0; i< WRENCH_STATE; i++){ _wrench[i] = 0.0;}
   // memcpy(_wrench, msg->wrench.quantity.data(), 6*sizeof(double));   
   is_valid_wrench = false;
  }

}

/* This function is to compute the torque input to the KUKA arm. 
This function takes in a trajectory and sampled at a low frequency and resamples it at 1000 Hz

*/
void LBRWrenchSineOverlayClient::is_command_active(const std_msgs::Bool::ConstPtr& msg) {
  command_active = msg->data;
}


/* 
Implementation of the computation of the velocity from finite difference and a window filtering.
*/
void LBRWrenchSineOverlayClient::computeVelocity(double curr_joint_pos[], double* vel_measured) {

  /*for (int i=0; i < n_pos_history-1; i++) {
      temp_joint_pos.col(i) = temp_joint_pos.col(i+1);  
  }*/
  // Shift the block to left
  temp_joint_pos.block(0,0,7,n_position_history-1) = temp_joint_pos.block(0,1,7,n_position_history-1).eval();

  // Append the current joint position to the end.
  temp_joint_pos.col(n_position_history-1) << curr_joint_pos[0], curr_joint_pos[1], curr_joint_pos[2], curr_joint_pos[3], curr_joint_pos[4], curr_joint_pos[5], curr_joint_pos[6]; 

  // Compute the velocity
  temp_joint_vel = temp_joint_pos.block(0,1,7,n_position_history-1) - temp_joint_pos.block(0,0,7,n_position_history-1);   

  // Filter the weights
  curr_velocity = temp_joint_vel * filter_weights;
  memcpy(vel_measured, curr_velocity.data(), 7*sizeof(curr_velocity.data()));

}
