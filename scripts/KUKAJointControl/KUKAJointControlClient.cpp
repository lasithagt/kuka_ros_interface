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
Copyright (C)  2014 
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

#include <models.hpp>
#include <kdl/framevel.hpp>

#include "KUKAJointControlClient.h"
#include "friLBRState.h"

#include <eigen3/Eigen/Core>
#include <boost/algorithm/cxx11/any_of.hpp> 

  // The classes
using ecl::CubicSpline;


#include <vector>


#ifndef M_PI
#define M_PI 3.14159265358979
#endif

//******************************************************************************
KUKAJointControl::KUKAJointControl(ros::NodeHandle &nh) 
   :command_active(false)
   ,is_valid(false)
   ,sample_time(0.0)
   ,is_command(false)
{
   printf("KUKA LBR initilized ...\n");


  // Subscribers
  sub_joint_position = nh.subscribe("/kuka/command/command_position", 1, &KUKAJointControl::getKUKAJointCmd, this);
  sub_command_active = nh.subscribe("/kuka/command/active", 1, &KUKAJointControl::is_command_active, this);

  // Publishers
  pub_torque       = nh.advertise<iiwa_msgs::JointTorque>("/kuka/state/KUKAActualTorque", 1); 
  pub_ext_torque   = nh.advertise<iiwa_msgs::JointTorque>("/kuka/state/KUKAExtTorque", 1); 
  pub_position     = nh.advertise<iiwa_msgs::JointPosition>("/kuka/state/KUKAJointPosition", 1); 
  pub_position_com = nh.advertise<iiwa_msgs::JointPosition>("/kuka/state/KUKAJointPositionCommand", 1); 
  pub_position_Ipo = nh.advertise<iiwa_msgs::JointPosition>("/kuka/state/KUKAJointPositionIpo", 1);
  joint_vel_pub_   = nh.advertise<iiwa_msgs::JointVelocity>("/kuka/state/KUKAJointVelocity", 1);   
  pub_kuka_time    = nh.advertise<std_msgs::Time>("/kuka/state/KUKATime", 1);

  // Initialization
  kukaTorque.torque.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaExtTorque.torque.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaPosition.position.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaPositionCommanded.position.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaPositionIpo.position.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaVelocity.velocity.quantity.resize(LBRState::NUMBER_OF_JOINTS);

  // Set current time to zero

  curr_velocity.resize(7);
  filter_weights.col(0) << 0.25/0.001, 0.25/0.001, 0.25/0.001, 0.25/0.001;
  n_pos_history = 5; // This is user defined.

  for (int i=0; i < n_pos_history; i++) {
      temp_joint_pos.col(i) << 0,-1.21026  ,0,-1.67354, 0,-0.463276, 0;    
  }

  temp_joint_vel = Eigen::MatrixXd::Zero(7, n_pos_history-1);

  // Initialize filters.
  const double cutoff_hz = 40;
  lp_filter = new DiscreteTimeLowPassFilter<double>(cutoff_hz, sample_time);

  }
  
//******************************************************************************
KUKAJointControl::~KUKAJointControl()
{
  delete(lp_filter);
}
      
//******************************************************************************
void KUKAJointControl::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // (re)initialize sine parameters when entering Monitoring
   switch (newState)
   {
      case MONITORING_READY:
      {
         sample_time = robotState().getSampleTime();
         ROS_INFO_STREAM(sample_time);
         break;
      }
      default:
      {
         break;
      }
   }
}
   
//******************************************************************************
void KUKAJointControl::command()
{
  mtx.lock();
  // calculate new offset
  double curr_joint_pos[LBRState::NUMBER_OF_JOINTS];
  double curr_velocity[LBRState::NUMBER_OF_JOINTS];

  memcpy(curr_joint_pos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  double joint_ext_torque[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_ext_torque, robotState().getExternalTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  double joint_measure_torque[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_measure_torque, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  double joint_commanded[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_commanded, robotState().getCommandedJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  double joint_Ipo[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_Ipo, robotState().getIpoJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));   

  curr_time.data.sec  = (robotState().getTimestampSec());
  curr_time.data.nsec = (robotState().getTimestampNanoSec());

  computeVelocity(curr_joint_pos, curr_velocity);

  // If the joint trajectory has been published, 
  publishState(joint_ext_torque, joint_measure_torque, curr_joint_pos, joint_Ipo, joint_commanded, curr_velocity, curr_time);

  // Set the position of the robot to desired.
  if (!is_valid && is_command==false) {
    memcpy(_positions, robotState().getCommandedJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
  } else {
    is_command = false;
  }

  robotCommand().setJointPosition(_positions);
  mtx.unlock();
}

// Implementation of the computation of the velocity from finite difference.
void KUKAJointControl::computeVelocity(double curr_joint_pos[], double* vel_measured) {
  /*for (int i=0; i < n_pos_history-1; i++) {
      temp_joint_pos.col(i) = temp_joint_pos.col(i+1);  
  }*/
  // Shift the block to left
  temp_joint_pos.block(0,0,7,n_pos_history-1) = temp_joint_pos.block(0,1,7,n_pos_history-1).eval();

  // Append the current joint position to the end.
  temp_joint_pos.col(n_pos_history-1) << curr_joint_pos[0], curr_joint_pos[1], curr_joint_pos[2], curr_joint_pos[3], curr_joint_pos[4], curr_joint_pos[5], curr_joint_pos[6]; 

  // Compute the velocity
  temp_joint_vel = temp_joint_pos.block(0,1,7,n_pos_history-1) - temp_joint_pos.block(0,0,7,n_pos_history-1);   

  // Filter the weights
  curr_velocity = temp_joint_vel * filter_weights;
  memcpy(vel_measured, curr_velocity.data(), 7*sizeof(curr_velocity.data()));

}

void KUKAJointControl::publishState(double jointExtTorque[], double jointTorque[], double jointState[], double jointStateIpo[], double jointStateCommanded[], double jointVelocity[], std_msgs::Time kuka_time)
{
  memcpy(kukaExtTorque.torque.quantity.data(), jointExtTorque, 7*sizeof(double));
  kukaExtTorque.header.stamp = ros::Time::now();

  memcpy(kukaTorque.torque.quantity.data(), jointTorque, 7*sizeof(double));
  kukaTorque.header.stamp = ros::Time::now();

  memcpy(kukaPosition.position.quantity.data(), jointState, 7*sizeof(double));
  kukaPosition.header.stamp = ros::Time::now();

  memcpy(kukaPositionIpo.position.quantity.data(), jointStateIpo, 7*sizeof(double));
  kukaPositionIpo.header.stamp = ros::Time::now();

  memcpy(kukaPositionCommanded.position.quantity.data(), jointStateCommanded, 7*sizeof(double));
  kukaPositionCommanded.header.stamp = ros::Time::now();

  memcpy(kukaVelocity.velocity.quantity.data(), jointVelocity, 7*sizeof(double));
  kukaVelocity.header.stamp = ros::Time::now();

  curr_time.data.sec = kuka_time.data.sec;
  curr_time.data.nsec = kuka_time.data.nsec;

  pub_torque.publish(kukaTorque);
  pub_ext_torque.publish(kukaExtTorque);
  pub_position.publish(kukaPosition);
  pub_position_Ipo.publish(kukaPositionIpo);
  pub_position_com.publish(kukaPositionCommanded);
  joint_vel_pub_.publish(kukaVelocity);
  pub_kuka_time.publish(curr_time);
}


// This function is to compute the torque input to the KUKA arm.
void KUKAJointControl::getKUKAJointCmd(const ros::MessageEvent<iiwa_msgs::JointPosition const>& event) { 

  is_command = true;
  is_valid = true;

  double time_diff = abs(event.getReceiptTime().toSec() - ros::Time::now().toSec());
  const iiwa_msgs::JointPosition::ConstPtr& msg = event.getMessage();

  for (int i = 0;i < LBRState::NUMBER_OF_JOINTS; i++) {
    if (abs(kukaPosition.position.quantity[i] - msg->position.quantity[i]) > 0.2) {
      is_valid = false;
    }

  }
  // bool ans = boost::algorithm::any_of(abs(kukaPosition.position.quantity - msg->position.quantity), isNVALID);

  if (is_valid==true && command_active==true && time_diff<0.1) {
    memcpy(_positions, msg->position.quantity.data(), 7*sizeof(double));
    
  } else {
    memcpy(_positions, kukaPositionCommanded.position.quantity.data(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    is_valid = false;
  }
  
}

void KUKAJointControl::is_command_active(const std_msgs::Bool::ConstPtr& msg) {
  command_active = msg->data;
}
