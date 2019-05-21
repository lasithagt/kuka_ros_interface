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
// #include <iiwa_ros/conversions.h>
// #include <kdl/chainjnttojacsolver.hpp>
// #include <kdl/chainjnttojacdotsolver.hpp>
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
LBRJointSineOverlayClient::LBRJointSineOverlayClient(ros::NodeHandle &nh) 
   : _active_point(0.0)
   ,_active_traj(0.0)
   ,_active_last_comm(false)
   ,_start_time(0.0)
{
   printf("KUKA LBR initilized ...\n");


  // Subscribers
  sub_joint_position_traj = nh.subscribe("/kuka/command/command_u_traj", 1, &LBRJointSineOverlayClient::getKUKAJointTrajCmd, this);
  sub_joint_position      = nh.subscribe("/kuka/command/command_position", 1, &LBRJointSineOverlayClient::getKUKAJointCmd, this);
  n_traj_points_sub       = nh.subscribe("/mpc/trajpoints", 10, &LBRJointSineOverlayClient::getTrajPoints, this);

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
  // const double cutoff_hz = 40;
  // vel_filters_.resize(
  // num_joints_, DiscreteTimeLowPassFilter<double>(cutoff_hz, kTimeStep));
  // utime_last_.resize(num_robots, -1);

  }
  
//******************************************************************************
LBRJointSineOverlayClient::~LBRJointSineOverlayClient()
{}
      
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
   if (_active_point == 1) {
    // Make this concise
     _positions[0] = _positions[0];
     _positions[1] = _positions[1];
     _positions[2] = _positions[2];
     _positions[3] = _positions[3];
     _positions[4] = _positions[4];
     _positions[5] = _positions[5];
     _positions[6] = _positions[6];
     std::cout << joint_commanded[0] << " " << joint_commanded[1] << " " << joint_commanded[2] << " " << joint_commanded[3] << " " << joint_commanded[4] <<  std::endl;
     std::cout << curr_joint_pos[0] << " " << curr_joint_pos[1] << " " << curr_joint_pos[2] << " " << curr_joint_pos[3] << " " << curr_joint_pos[4] <<  std::endl; 
   } else {
      memcpy(_positions, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
   }

   // Set the position of the robot to desired.
   robotCommand().setJointPosition(_positions);
}

// Implementation of the computation of the velocity from finite difference.
void LBRJointSineOverlayClient::computeVelocity(double curr_joint_pos[], double* vel_measured) {
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

void LBRJointSineOverlayClient::publishState(double jointExtTorque[], double jointTorque[], double jointState[], double jointStateIpo[], double jointStateCommanded[], double jointVelocity[], std_msgs::Time kuka_time)
{
  // kukaExtTorque.torque.time_from_start = ros::Time::now();
  memcpy(kukaExtTorque.torque.quantity.data(), jointExtTorque, 7*sizeof(double));
  kukaExtTorque.header.stamp.sec = kuka_time.data.sec;
  kukaExtTorque.header.stamp.nsec = kuka_time.data.nsec;

  memcpy(kukaTorque.torque.quantity.data(), jointTorque, 7*sizeof(double));
  kukaTorque.header.stamp.sec = kuka_time.data.sec;
  kukaTorque.header.stamp.nsec = kuka_time.data.nsec;

  memcpy(kukaPosition.position.quantity.data(), jointState, 7*sizeof(double));
  kukaPosition.header.stamp.sec = kuka_time.data.sec;
  kukaPosition.header.stamp.nsec = kuka_time.data.nsec;

  memcpy(kukaPositionIpo.position.quantity.data(), jointStateIpo, 7*sizeof(double));
  kukaPositionIpo.header.stamp.sec = kuka_time.data.sec;
  kukaPositionIpo.header.stamp.nsec = kuka_time.data.nsec;

  memcpy(kukaPositionCommanded.position.quantity.data(), jointStateCommanded, 7*sizeof(double));
  kukaPositionCommanded.header.stamp.sec = kuka_time.data.sec;
  kukaPositionCommanded.header.stamp.nsec = kuka_time.data.nsec;

      // Velocity filtering.
    /*if (robot_dt != 0.) {
      for (int i = 0; i < kNumJoints; i++) {
        const int index = joint_offset + i;
        const double q_diff = state.getMeasuredJointPosition()[i] -
                              lcm_status_.joint_position_measured[index];
        lcm_status_.joint_velocity_estimated[index] =
            vel_filters_[index].filter(q_diff / robot_dt);
}*/

  memcpy(kukaVelocity.velocity.quantity.data(), jointVelocity, 7*sizeof(double));
  kukaVelocity.header.stamp.sec = kuka_time.data.sec;
  kukaVelocity.header.stamp.nsec = kuka_time.data.nsec;

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
void LBRJointSineOverlayClient::getKUKAJointCmd(const iiwa_msgs::JointPosition::ConstPtr& msg) {
  _active_point = 1;
  
  memcpy(_positions, msg->position.quantity.data(), 7*sizeof(double));
  
}

// This function is to compute the torque input to the KUKA arm.
void LBRJointSineOverlayClient::getKUKAJointTrajCmd(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {

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
  
  /*if (t.size() == 1) {
    _active_traj = 0;
    _active_last_comm = true;
  } else {
    _active_traj = 1;
  }*/

  if (n_traj_points-2 == msg->header.seq) {
    ROS_INFO_STREAM("Last" <<  std::endl);
    _active_traj = false;
    _active_last_comm = true;
  } else {
    _active_traj = true;
  }
  _start_time = (double)msg->header.stamp.sec + (double)(msg->header.stamp.nsec) * pow(10,-9);

}

// Set the inital states of the kuka_env states (first 17 states)
void LBRJointSineOverlayClient::set_init_state(Eigen::VectorXd inits_kuka_env) {
    
    for (int i=0; i < n_pos_history; i++) {
      temp_joint_pos.col(i) = (inits_kuka_env.array()).segment(0,7);    
    }
}

void LBRJointSineOverlayClient::getTrajPoints(const std_msgs::Float32::ConstPtr& msg) {
    n_traj_points = (double)msg->data;
}
