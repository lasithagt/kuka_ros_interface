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

#include "LBRTorqueSineOverlayClient.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

//******************************************************************************
LBRTorqueSineOverlayClient::LBRTorqueSineOverlayClient(ros::NodeHandle &nh) 
   :command_active(false)
   ,is_valid(false)
   ,is_command(false)
   ,joint_cmd_active(false)
{
  ROS_INFO_STREAM("KUKA LBR initialized ...\n");

  // Subscribers
  sub_command_torque = nh.subscribe("/kuka/command/command_torque", 1, &LBRTorqueSineOverlayClient::getKUKATorqueCmd, this);
  sub_command_joint  = nh.subscribe("/kuka/command/command_position", 1, &LBRTorqueSineOverlayClient::getKUKAJointCmd, this);
  sub_command_active = nh.subscribe("/kuka/command/active", 1, &LBRTorqueSineOverlayClient::is_command_active, this);

  // Publishers
  pub_torque         = nh.advertise<iiwa_msgs::JointTorque>("/kuka/state/KUKAActualTorque", 1);
  pub_ext_torque     = nh.advertise<iiwa_msgs::JointTorque>("/kuka/state/KUKAExtTorque", 1);
  pub_position       = nh.advertise<iiwa_msgs::JointPosition>("/kuka/state/KUKAJointPosition", 1);
  pub_position_com   = nh.advertise<iiwa_msgs::JointPosition>("/kuka/state/KUKAJointPositionCommand", 1);
  pub_velocity       = nh.advertise<iiwa_msgs::JointVelocity>("/kuka/state/KUKAJointVelocity", 1);
  pub_kuka_time      = nh.advertise<std_msgs::Time>("/kuka/state/KUKATime", 1);
  
  // Set current time to zero
  kukaTorque.torque.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaExtTorque.torque.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaPosition.position.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaPositionCommanded.position.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaVelocity.velocity.quantity.resize(LBRState::NUMBER_OF_JOINTS);

  // initialize the torque
  for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ torques_[i] = 0.0;}


  // get sample time
  // sample_time = 0.001; // robotState().getSampleTime();



}

//******************************************************************************
LBRTorqueSineOverlayClient::~LBRTorqueSineOverlayClient() = default;

//******************************************************************************
void LBRTorqueSineOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // (re)initialize sine parameters when entering Monitoring
   switch (newState)
   {
      case MONITORING_READY:
      {
        sample_time = robotState().getSampleTime();

        // Initialize filters.
        for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++) { 
          torques_[i] = 0.0;
        }
         break;
      }
      default:
      {
         break;
      }
   }
}
//******************************************************************************

void LBRTorqueSineOverlayClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
   // the base method.
   LBRClient::waitForCommand();
   
   // If we want to command torques, we have to command them all the time; even in
   // waitForCommand(). This has to be done due to consistency checks. In this state it is 
   // only necessary, that some torque values are sent. The LBR does not take the 
   // specific value into account.
   if (robotState().getClientCommandMode() == TORQUE)
   {
      robotCommand().setTorque(torques_);
   }
}

void LBRTorqueSineOverlayClient::monitor() {
   LBRClient::monitor();

}
/*
 * Publish the robot state to ros topics
 */
void LBRTorqueSineOverlayClient::publishState(const State& robotState, std_msgs::Time kuka_time)
{

  memcpy(kukaExtTorque.torque.quantity.data(), robotState.joint_ext_torque, 7*sizeof(double));
  kukaExtTorque.header.stamp = ros::Time::now();

  memcpy(kukaTorque.torque.quantity.data(), robotState.joint_measure_torque, 7*sizeof(double));
  kukaTorque.header.stamp = ros::Time::now();

  memcpy(kukaPosition.position.quantity.data(), robotState.current_joint_pos, 7*sizeof(double));
  kukaPosition.header.stamp = ros::Time::now();

  memcpy(kukaPositionCommanded.position.quantity.data(), robotState.joint_commanded, 7*sizeof(double));
  kukaPositionCommanded.header.stamp = ros::Time::now();

  memcpy(kukaVelocity.velocity.quantity.data(), robotState.current_joint_velocity, 7*sizeof(double));
  kukaVelocity.header.stamp = ros::Time::now();

  curr_time.data.sec = kuka_time.data.sec;
  curr_time.data.nsec = kuka_time.data.nsec;

  pub_torque.publish(kukaTorque);
  pub_ext_torque.publish(kukaExtTorque);
  pub_position.publish(kukaPosition);
  pub_position_com.publish(kukaPositionCommanded);
  pub_velocity.publish(kukaVelocity);
  pub_kuka_time.publish(curr_time);

}
//******************************************************************************
void LBRTorqueSineOverlayClient::command()
{
  // In command(), the joint values have to be sent. Which is done by calling
  // the base method.
  mtx.lock();

  // Copy current joint position to previous joint positions
  for (int i = 0;i < LBRState::NUMBER_OF_JOINTS;i++) {kukaState.previous_joint_pos[i] = kukaState.current_joint_pos[i];}

  // Get the current joint positions
  memcpy(kukaState.current_joint_pos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  // Measured torque = Commanded torque + External Torque;
  memcpy(kukaState.joint_measure_torque, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  // Get the commanded torque
  memcpy(kukaState.joint_commanded, robotState().getCommandedTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  // Get the external torque on joints
  memcpy(kukaState.joint_ext_torque, robotState().getExternalTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  curr_time.data.sec  = (robotState().getTimestampSec());
  curr_time.data.nsec = (robotState().getTimestampNanoSec());
  
  // compute the velocity
  computeVelocity(kukaState);

  // publish the states
  publishState(kukaState, curr_time);

  // Check for current joint position an
  if (robotState().getClientCommandMode() == TORQUE) { 

    // Set superposed joint torques
    if (joint_cmd_active) {
      robotCommand().setJointPosition(jointPositionsCommand);
    } else {
      robotCommand().setJointPosition(kukaState.current_joint_pos);
    }
  
    robotCommand().setTorque(torques_);
  }
  mtx.unlock();
}

/* 
Implementation of the computation of the velocity from finite difference and a window filtering.
*/
void LBRTorqueSineOverlayClient::computeVelocity(State& robotState) {
    // estimate the velocity

    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++)
    {
        robotState.joint_velocity_raw.at(i) = robotState.joint_velocity_filtered.at(i);
        auto temp = (robotState.current_joint_pos[i] - robotState.previous_joint_pos[i]) / sample_time;
        robotState.joint_velocity_filtered.at(i) = temp;
    }

    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++)
    {
        robotState.current_joint_velocity[i] = 0.99*robotState.joint_velocity_filtered.at(i) + 0.01*robotState.joint_velocity_raw.at(i);
    }
}

// This function is to compute the joint position input to the KUKA arm.
void LBRTorqueSineOverlayClient::getKUKAJointCmd(const ros::MessageEvent<iiwa_msgs::JointPosition const>& event) {

  is_command = true;
  is_valid = true;
  joint_cmd_active = true;

  double time_diff = abs(event.getReceiptTime().toSec() - ros::Time::now().toSec());
  const iiwa_msgs::JointPosition::ConstPtr& msg = event.getMessage();

  for (int i = 0; i<LBRState::NUMBER_OF_JOINTS;i++)
  {
      jointPositionsCommand[i] = msg->position.quantity.at(i);
  }


}

// This function is to compute the torque input to the KUKA arm.
void LBRTorqueSineOverlayClient::getKUKATorqueCmd(const ros::MessageEvent<iiwa_msgs::JointTorque const>& event) {

  is_command = true;
  is_valid = true;

  double time_diff = abs(event.getReceiptTime().toSec() - ros::Time::now().toSec());
  const iiwa_msgs::JointTorque::ConstPtr& msg = event.getMessage();

  for (int i = 0;i < LBRState::NUMBER_OF_JOINTS;i++)
  {
      torques_[i] = msg->torque.quantity.at(i);
  }
  
}

/* This function is to compute the torque input to the KUKA arm. 
This function takes in a trajectory and sampled at a low frequency and resamples it at 1000 Hz
*/
void LBRTorqueSineOverlayClient::is_command_active(const std_msgs::Bool::ConstPtr& msg) {
  command_active = msg->data;
}

bool LBRTorqueSineOverlayClient::isSafe()
{

}

