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
#include <iiwa_ros.h>
#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <iiwa_ros/conversions.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <models.hpp>
#include <kdl/framevel.hpp>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "LBRJointSineOverlayClient.h"
#include "friLBRState.h"

#include <eigen3/Eigen/Core>

  // The classes
using ecl::CubicSpline;


#include <vector>


#ifndef M_PI
#define M_PI 3.14159265358979
#endif

//******************************************************************************
LBRJointSineOverlayClient::LBRJointSineOverlayClient(unsigned int jointMask, 
      double freqHz, double amplRad, double filterCoeff,  ros::NodeHandle &nh) 
   : _jointMask(jointMask)
   , _freqHz(freqHz)
   , _amplRad(amplRad)
   , _filterCoeff(filterCoeff)
   , _offset(0.0)
   , _phi(0.0)
   , _stepWidth(0.0)
   ,_active_point(0.0)
   ,_active_traj(0.0)
   ,_start_time(0.0)
{
   printf("LBRJointSineOverlayClient initialized:\n"
         "\tjoint mask: 0x%x\n"
         "\tfrequency (Hz): %f\n"
         "\tamplitude (rad): %f\n"
         "\tfilterCoeff: %f\n",
         jointMask, freqHz, amplRad, filterCoeff);

// Subscribers
sub_joint_position_traj = nh.subscribe("/kuka/KUKACmdTrajPosition", 1, &LBRJointSineOverlayClient::getKUKAJointTrajCmd, this);
sub_joint_position = nh.subscribe("/user/KUKACmdPosition", 1, &LBRJointSineOverlayClient::getKUKAJointCmd, this);

// Publishers
pub_torque = nh.advertise<iiwa_msgs::JointTorque>("/kuka/KUKAActualTorque", 1); 
pub_ext_torque = nh.advertise<iiwa_msgs::JointTorque>("/kuka/KUKAExtTorque", 1); 
pub_position = nh.advertise<iiwa_msgs::JointPosition>("/kuka/KUKAJointPosition", 1); 
pub_position_com = nh.advertise<iiwa_msgs::JointPosition>("/kuka/KUKAJointPositionCommand", 1); 

// Set current time to zero
curr_time = ros::Time();

}
//******************************************************************************
LBRJointSineOverlayClient::~LBRJointSineOverlayClient()
{
}
      
//******************************************************************************
void LBRJointSineOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // (re)initialize sine parameters when entering Monitoring
   switch (newState)
   {
      case MONITORING_READY:
      {
         ROS_INFO_STREAM(robotState().getSampleTime());
         break;
      }
      default:
      {
         break;
      }
   }
}
   
//******************************************************************************
void LBRJointSineOverlayClient::command()
{
   // calculate new offset

   // add offset to ipo joint position for all masked joints
   double jointPos[LBRState::NUMBER_OF_JOINTS];
   memcpy(jointPos, robotState().getIpoJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
   
   double jointExtTorque[LBRState::NUMBER_OF_JOINTS];
   memcpy(jointExtTorque, robotState().getExternalTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

   double jointMeasureTorque[LBRState::NUMBER_OF_JOINTS];
   memcpy(jointMeasureTorque, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

   double jointCommanded[LBRState::NUMBER_OF_JOINTS];
   memcpy(jointCommanded, robotState().getCommandedJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

   double t = curr_time.now().toSec() - _start_time; // Get the current time.

   publishState(jointExtTorque, jointMeasureTorque, jointPos, jointCommanded);
   // If the joint trajectory has been published, 

   if (_active_traj == 1) {

     _positions[0] = sp_j1(t);
     _positions[1] = sp_j2(t);
     _positions[2] = sp_j3(t);
     _positions[3] = sp_j4(t);
     _positions[4] = sp_j5(t);
     _positions[5] = sp_j6(t);
     _positions[6] = sp_j7(t);

   } else {
      memcpy(_positions, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
   }

   /*if (_active_point == 0) {
     memcpy(_positions, jointPos, LBRState::NUMBER_OF_JOINTS * sizeof(double));
   }*/

   robotCommand().setJointPosition(_positions);
}


void LBRJointSineOverlayClient::publishState(double jointExtTorque[], double jointTorque[], double jointState[], double jointStateCommanded[])
{
  iiwa_msgs::JointTorque kukaTorque;
  iiwa_msgs::JointTorque kukaExtTorque;
  iiwa_msgs::JointPosition kukaPosition;
  iiwa_msgs::JointPosition kukaPositionCommanded;

  kukaExtTorque.header.stamp = ros::Time::now();
  kukaExtTorque.torque.a1 = jointExtTorque[0];
  kukaExtTorque.torque.a2 = jointExtTorque[1];
  kukaExtTorque.torque.a3 = jointExtTorque[2];
  kukaExtTorque.torque.a4 = jointExtTorque[3];
  kukaExtTorque.torque.a5 = jointExtTorque[4];
  kukaExtTorque.torque.a6 = jointExtTorque[5];
  kukaExtTorque.torque.a7 = jointExtTorque[6];

  kukaTorque.header.stamp = ros::Time::now();
  kukaTorque.torque.a1 = jointTorque[0];
  kukaTorque.torque.a2 = jointTorque[1];
  kukaTorque.torque.a3 = jointTorque[2];
  kukaTorque.torque.a4 = jointTorque[3];
  kukaTorque.torque.a5 = jointTorque[4];
  kukaTorque.torque.a6 = jointTorque[5];
  kukaTorque.torque.a7 = jointTorque[6];

  kukaPosition.header.stamp = ros::Time::now();
  kukaPosition.position.a1 = jointState[0];
  kukaPosition.position.a2 = jointState[1];
  kukaPosition.position.a3 = jointState[2];
  kukaPosition.position.a4 = jointState[3];
  kukaPosition.position.a5 = jointState[4];
  kukaPosition.position.a6 = jointState[5];
  kukaPosition.position.a7 = jointState[6];

  kukaPositionCommanded.header.stamp = ros::Time::now();
  kukaPositionCommanded.position.a1 = jointStateCommanded[0];
  kukaPositionCommanded.position.a2 = jointStateCommanded[1];
  kukaPositionCommanded.position.a3 = jointStateCommanded[2];
  kukaPositionCommanded.position.a4 = jointStateCommanded[3];
  kukaPositionCommanded.position.a5 = jointStateCommanded[4];
  kukaPositionCommanded.position.a6 = jointStateCommanded[5];
  kukaPositionCommanded.position.a7 = jointStateCommanded[6];

  pub_torque.publish(kukaTorque);
  pub_ext_torque.publish(kukaExtTorque);
  pub_position.publish(kukaPosition);
  pub_position_com.publish(kukaPositionCommanded);

}


// This function is to compute the torque input to the KUKA arm.
void LBRJointSineOverlayClient::getKUKAJointCmd(const iiwa_msgs::JointPosition::ConstPtr& msg) {
  _active_point = 1;
  _positions[0] = msg->position.a1;
  _positions[1] = msg->position.a2;
  _positions[2] = msg->position.a3;
  _positions[3] = msg->position.a4;
  _positions[4] = msg->position.a5;
  _positions[5] = msg->position.a6;
  _positions[6] = msg->position.a7;
}



// This function is to compute the torque input to the KUKA arm.
void LBRJointSineOverlayClient::getKUKAJointTrajCmd(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {

   // Toggle this to get traj command activated
   
   _start_time = curr_time.now().toSec();


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
        t[i]  = original_freq*i; //msg->points.at(i).time_from_start.sec + (double)msg->points.at(i).time_from_start.nsec * pow(10,-9);
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

  // There is a delay that you need to take into account in here.
  // std::cout << "Interp: " << sp_j1(curr_time.now().toSec() - _start_time) << std::endl;
  // std::cout << t << std::endl;
  // std::cout << j1 << std::endl;
  // std::cout << "Interp: " << sp_j1(curr_time.now().toSec() - _start_time) << std::endl;


  if (t.size() == 1) {
    _active_traj = 0;
  } else {
    _active_traj = 1;
  }

}
