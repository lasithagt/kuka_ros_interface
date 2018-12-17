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

#include <models.hpp>

// #include <fstream>
// #include <iostream>

#include "LBRTorqueSineOverlayClient.h"
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/framevel.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

//******************************************************************************
LBRTorqueSineOverlayClient::LBRTorqueSineOverlayClient(unsigned int jointMask, 
      double freqHz, double torqueAmplitude, ros::NodeHandle &nh) 
   :_jointMask(jointMask)
   , _freqHz(freqHz)
   , _torqueAmpl(torqueAmplitude)
   , _phi(0.0)
   , _stepWidth(0.0)
   , _sampleTime(0.0)
{
  printf("LBRTorqueSineOverlayClient initialized:\n");

  // Subscribers
  sub_command_torque = nh.subscribe("/kuka/command/command_torque", 1, &LBRTorqueSineOverlayClient::getKUKATorque, this);
  // sub_joint_position      = nh.subscribe("/user/KUKACmdPosition", 1, &LBRTorqueSineOverlayClient::getKUKATorqueCmd, this);

  // Publishers
  pub_torque       = nh.advertise<iiwa_msgs::JointTorque>("/kuka/state/KUKAActualTorque", 1); 
  pub_ext_torque   = nh.advertise<iiwa_msgs::JointTorque>("/kuka/state/KUKAExtTorque", 1); 
  pub_position     = nh.advertise<iiwa_msgs::JointPosition>("/kuka/state/KUKAJointPosition", 1); 
  pub_position_com = nh.advertise<iiwa_msgs::JointPosition>("/kuka/state/KUKAJointPositionCommand", 1); 
  joint_vel_pub_   = nh.advertise<iiwa_msgs::JointVelocity>("/kuka/state/KUKAJointVelocity", 1);   
  pub_kuka_time    = nh.advertise<std_msgs::Time>("/kuka/state/KUKATime", 1);
  
  // Set current time to zero
  // curr_time = ros::Time();
  kuka_time = 0.0;

  // curr_velocity.resize(7);
  filter_weights.col(0) << 0.25/0.001, 0.25/0.001, 0.25/0.001, 0.25/0.001;
  
  // Number of points to the moving average
  n_position_history = 5;

  temp_joint_vel = Eigen::MatrixXd::Zero(7, n_position_history-1);
  init_velocity_flag = true;

  kukaTorque.torque.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaExtTorque.torque.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaPosition.position.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaPositionCommanded.position.quantity.resize(LBRState::NUMBER_OF_JOINTS);
  kukaVelocity.velocity.quantity.resize(LBRState::NUMBER_OF_JOINTS);

  for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}

}

//******************************************************************************
LBRTorqueSineOverlayClient::~LBRTorqueSineOverlayClient()
{

}


      
//******************************************************************************
void LBRTorqueSineOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // (re)initialize sine parameters when entering Monitoring
   switch (newState)
   {
      case MONITORING_READY:
      {
         for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}
         _phi = 0.0;
         _sampleTime = robotState().getSampleTime();
         _stepWidth = 2 * M_PI * _freqHz * robotState().getSampleTime();
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
      robotCommand().setTorque(_torques);
      
   }
}

void LBRTorqueSineOverlayClient::monitor() {
   LBRClient::monitor();

}


void LBRTorqueSineOverlayClient::publishState(double jointExtTorque[], double jointTorque[], double jointState[], double jointStateCommanded[], double jointVelocity[], std_msgs::Time kuka_time)
{
  /*iiwa_msgs::JointTorque kukaTorque;
  iiwa_msgs::JointTorque kukaExtTorque;
  iiwa_msgs::JointPosition kukaPosition;
  iiwa_msgs::JointPosition kukaPositionCommanded;
  iiwa_msgs::JointVelocity kukaVelocity;*/


  // kukaExtTorque.torque.time_from_start = ros::Time::now();
  memcpy(kukaExtTorque.torque.quantity.data(), jointExtTorque, 7*sizeof(double));
  memcpy(kukaTorque.torque.quantity.data(), jointTorque, 7*sizeof(double));
  memcpy(kukaPosition.position.quantity.data(), jointState, 7*sizeof(double));
  memcpy(kukaPositionCommanded.position.quantity.data(), jointStateCommanded, 7*sizeof(double));
  memcpy(kukaVelocity.velocity.quantity.data(), jointVelocity, 7*sizeof(double));
  curr_time.data.sec = kuka_time.data.sec;
  curr_time.data.nsec = kuka_time.data.nsec;

  pub_torque.publish(kukaTorque);
  pub_ext_torque.publish(kukaExtTorque);
  pub_position.publish(kukaPosition);
  pub_position_com.publish(kukaPositionCommanded);
  joint_vel_pub_.publish(kukaVelocity);
  pub_kuka_time.publish(curr_time);

  // Subscribers

}
//******************************************************************************
void LBRTorqueSineOverlayClient::command()
{
  // In command(), the joint values have to be sent. Which is done by calling
  // the base method.
  LBRClient::command();

  

  double curr_joint_pos[LBRState::NUMBER_OF_JOINTS];
  double curr_velocity[LBRState::NUMBER_OF_JOINTS];
  std_msgs::Time curr_time;

  double jointPosDes[LBRState::NUMBER_OF_JOINTS];
  jointPosDes[0] = 90.0* M_PI/180; jointPosDes[1] = -60.0* M_PI/180; jointPosDes[2] = 0.0* M_PI/180; jointPosDes[3] = 60.0* M_PI/180; jointPosDes[4] = 0.0* M_PI/180; jointPosDes[5] = -60.0* M_PI/180; jointPosDes[6] = 0.0* M_PI/180;
  

  memcpy(curr_joint_pos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
  double _offset = 20.0* M_PI/180;

  // Measured torque = Commanded torque + External Torque;
  double joint_measure_torque[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_measure_torque, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  // Commanded torque
  double joint_commanded[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_commanded, robotState().getCommandedTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  // External toruqe on joints
  double joint_ext_torque[LBRState::NUMBER_OF_JOINTS];
  memcpy(joint_ext_torque, robotState().getExternalTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

  curr_time.data.sec  = (robotState().getTimestampSec());
  curr_time.data.nsec = (robotState().getTimestampNanoSec());
  
  // get KUKA time
  // kuka_time = (double)robotState().getTimestampSec() + ((double)robotState().getTimestampNanoSec())/(1e9);
  
  computeVelocity(curr_joint_pos, curr_velocity);

  publishState(joint_ext_torque, joint_measure_torque, curr_joint_pos, joint_commanded, curr_velocity, curr_time);

  // Check for correct ClientCommandMode.
  robotCommand().setJointPosition(curr_joint_pos);
  if (robotState().getClientCommandMode() == TORQUE)
  { 
     
    for (int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
    {
      if (_jointMask & (1<<i))
      { 
          // _torques[i] = 200*(jointPosDes[i] - curr_joint_pos[i]); // offset;
          curr_joint_pos[i] = curr_joint_pos[i]; // _offset;
      }
    }

  // Set superposed joint torques.

  robotCommand().setTorque(_torques);
  // robotCommand().setJointPosition(curr_joint_pos);

  }
}

/* 
Implementation of the computation of the velocity from finite difference and a window filtering.
*/
void LBRTorqueSineOverlayClient::computeVelocity(double curr_joint_pos[], double* vel_measured) {

  if (init_velocity_flag) {

    for (int i=0; i < n_position_history; i++) {
        temp_joint_pos.col(i) << curr_joint_pos[0], curr_joint_pos[1], curr_joint_pos[2], curr_joint_pos[3], curr_joint_pos[4], curr_joint_pos[5], curr_joint_pos[6];    
    } 

    init_velocity_flag = false;
  }

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

// This function is to compute the torque input to the KUKA arm.
void LBRTorqueSineOverlayClient::getKUKATorque(const iiwa_msgs::JointTorque::ConstPtr& msg) {
  _torques[0] = msg->torque.quantity.at(0);
  _torques[1] = msg->torque.quantity.at(1);
  _torques[2] = msg->torque.quantity.at(2);
  _torques[3] = msg->torque.quantity.at(3);
  _torques[4] = msg->torque.quantity.at(4);
  _torques[5] = msg->torque.quantity.at(5);
  _torques[6] = msg->torque.quantity.at(6);

}

/* This function is to compute the torque input to the KUKA arm. 
This function takes in a trajectory and sampled at a low frequency and resamples it at 1000 Hz

*/
void LBRTorqueSineOverlayClient::getKUKATorqueTrajCmd(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {

   // Toggle this to get traj command activated
   
   int n_len = msg->points.size();

   ecl::Array<double> t(n_len);
   ecl::Array<double> j1(n_len);
   ecl::Array<double> j2(n_len);
   ecl::Array<double> j3(n_len);
   ecl::Array<double> j4(n_len);
   ecl::Array<double> j5(n_len);
   ecl::Array<double> j6(n_len);
   ecl::Array<double> j7(n_len);

   double original_freq = 0.02; // The rate at which controls are sent out.

   if (n_len > 1) {
     for (int i = 0; i < n_len; i++) {

        t[i]  = original_freq * i;                       // msg->points.at(i).time_from_start.sec + (double)msg->points.at(i).time_from_start.nsec * pow(10,-9);
        j1[i] = msg->points.at(i).positions.at(1);
        j2[i] = msg->points.at(i).positions.at(2);
        j3[i] = msg->points.at(i).positions.at(3);
        j4[i] = msg->points.at(i).positions.at(4);
        j5[i] = msg->points.at(i).positions.at(5);
        j6[i] = msg->points.at(i).positions.at(6);
        j7[i] = msg->points.at(i).positions.at(7);
     }

    sp_j1 = ecl::CubicSpline::ContinuousDerivatives(t,j1,0,0);
    sp_j2 = ecl::CubicSpline::ContinuousDerivatives(t,j2,0,0);
    sp_j3 = ecl::CubicSpline::ContinuousDerivatives(t,j3,0,0);
    sp_j4 = ecl::CubicSpline::ContinuousDerivatives(t,j4,0,0);
    sp_j5 = ecl::CubicSpline::ContinuousDerivatives(t,j5,0,0);
    sp_j6 = ecl::CubicSpline::ContinuousDerivatives(t,j6,0,0);
    sp_j7 = ecl::CubicSpline::ContinuousDerivatives(t,j7,0,0);
    
  }
}

