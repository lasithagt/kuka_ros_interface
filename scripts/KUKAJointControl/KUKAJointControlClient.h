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
#ifndef KUKAJOINTCONTROL_H
#define KUKAJOINTCONTROL_H

#include "friLBRClient.h"
#include <ros/ros.h>

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>

#include <kdl/jntarrayvel.hpp>

#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>

#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>
#include <ecl/exceptions.hpp>
#include <ecl/errors.hpp>
#include <ecl/concepts.hpp>
#include <ecl/converters.hpp>
#include <mutex>

#include "low_pass.hpp"

using namespace KUKA::FRI;

/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class KUKAJointControl : public LBRClient
{
   
public:
      
   /**
    * \brief Constructor.
    * 
    * @param jointMask Bitmask that encodes the joint indices to be overlayed by sine waves
    * @param freqHz Sine frequency in hertz
    * @param amplRad Sine amplitude in radians
    * @param filterCoeff Filter coefficient between 0 (filter off) and 1 (max filter)
    */
   KUKAJointControl(ros::NodeHandle &nh);
   
   /** 
    * \brief Destructor.
    */
   ~KUKAJointControl();
   
   /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(ESessionState oldState, ESessionState newState);

   virtual void publishState(double jointExtTorque[], double jointTorque[], double jointState[], double jointIpoState[], double jointCommanded[], double jointVelocity[], std_msgs::Time kuka_time);


   virtual void getKUKAJointCmd(const ros::MessageEvent<iiwa_msgs::JointPosition const>& msg);
   
   void computeVelocity(double curr_joint_pos[], double* vel_measured);

   void is_command_active(const std_msgs::Bool::ConstPtr& msg); 
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   virtual void command();
      
private:
   
   double _positions[LBRState::NUMBER_OF_JOINTS]; //!< commanded superposed torques
   bool is_valid;
   bool command_active;
   bool is_command;

   ros::Subscriber sub_joint_position;
   ros::Subscriber sub_joint_position_traj;
   ros::Subscriber n_traj_points_sub;
   ros::Subscriber sub_command_active;

   ros::Publisher pub_position;
   ros::Publisher pub_position_com;
   ros::Publisher pub_position_Ipo;
   ros::Publisher pub_torque;
   ros::Publisher pub_ext_torque;
   ros::Publisher joint_vel_pub_;
   ros::Publisher pub_kuka_time;

   std_msgs::Time curr_time;

   double sample_time;
   int n_pos_history;

   iiwa_msgs::JointTorque kukaTorque;
   iiwa_msgs::JointTorque kukaExtTorque;
   iiwa_msgs::JointPosition kukaPosition;
   iiwa_msgs::JointPosition kukaPositionIpo;
   iiwa_msgs::JointPosition kukaPositionCommanded;
   iiwa_msgs::JointVelocity kukaVelocity;

   DiscreteTimeLowPassFilter<double>* lp_filter;


   Eigen::Matrix<double, 7, 5> temp_joint_pos;
   Eigen::Matrix<double, 7, 4> temp_joint_vel;
   
   Eigen::Matrix<double,4,1> filter_weights;
   Eigen::VectorXd curr_velocity;

   std::mutex mtx;

};

#endif // _KUKA_FRI_LBR_JOINT_SINE_OVERLAY_CLIENT_H
