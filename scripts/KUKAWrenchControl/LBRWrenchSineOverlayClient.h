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
#ifndef _KUKA_FRI_LBR_WRENCH_SINE_OVERLAY_CLIENT_H
#define _KUKA_FRI_LBR_WRENCH_SINE_OVERLAY_CLIENT_H


#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/Wrench.h>
#include <iiwa_msgs/JointPositionVelocity.h>

#include "friLBRClient.h"
#include <ros/ros.h>
#include <models.hpp>

#include <eigen3/Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>


#include <mutex>
#define WRENCH_STATE 6


#include "low_pass.hpp"

using namespace KUKA::FRI;

/**
 * \brief Test client that can overlay interpolator joint positions with sine waves.
 */
class LBRWrenchSineOverlayClient : public LBRClient
{
   
public:
      
   /**
    * \brief Constructor.
    * 
    * @param freqHzX Sine frequency in hertz of force in X-direction
    * @param freqHzY Sine frequency in hertz of force in Y-direction
    * @param amplRadX Sine amplitude in radians of force in X-direction
    * @param amplRadY Sine amplitude in radians of force in Y-direction
    */
   LBRWrenchSineOverlayClient(ros::NodeHandle &nh);
   
   /** 
    * \brief Destructor.
    */
   ~LBRWrenchSineOverlayClient();
   
   /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(ESessionState oldState, ESessionState newState);
   
   /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    */
   virtual void waitForCommand();
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   virtual void command();

   virtual void publishState(double jointExtTorque[], double jointTorque[], double jointState[], double jointStateCommanded[], double jointVelocity[], std_msgs::Time t);

   virtual void getKUKAWrenchCmd(const ros::MessageEvent<iiwa_msgs::Wrench const>& event);

   virtual void getKUKAJointCmd(const ros::MessageEvent<iiwa_msgs::JointPosition const>& event);

   void is_command_active(const std_msgs::Bool::ConstPtr& msg); 

   void computeVelocity(double curr_joint_pos[], double* vel_measured);
      
private:
   double _wrench[WRENCH_STATE]; //!< commanded superposed torques
   double _joints[LBRState::NUMBER_OF_JOINTS];
   double sample_time;

   bool is_valid;
   bool command_active;
   bool is_valid_wrench;
   bool is_command_wrench;
   bool is_command;
   bool joint_cmd_active;


   ros::Subscriber sub_command_wrench;
   ros::Subscriber sub_command_joint;
   ros::Subscriber sub_joint_position;
   // ros::Subscriber sub_joint_position_traj;
   ros::Subscriber sub_command_active;

   ros::Publisher pub_position;
   ros::Publisher pub_position_com;
   ros::Publisher pub_torque;
   ros::Publisher pub_ext_torque;
   ros::Publisher joint_vel_pub_;
   ros::Publisher pub_kuka_time;

   int n_position_history;

   std_msgs::Time curr_time;

   iiwa_msgs::JointTorque kukaTorque;
   iiwa_msgs::JointTorque kukaExtTorque;
   iiwa_msgs::JointPosition kukaPosition;
   iiwa_msgs::JointPosition kukaPositionCommanded;
   iiwa_msgs::JointVelocity kukaVelocity;

   DiscreteTimeLowPassFilter<double>* lp_filter;


   Eigen::Matrix<double, 7, 5> temp_joint_pos;
   Eigen::Matrix<double, 7, 4> temp_joint_vel;
   
   Eigen::Matrix<double,4,1> filter_weights;
   Eigen::Matrix<double,7,1> curr_velocity;

   std::mutex mtx;
   
};

#endif // _KUKA_FRI_LBR_WRENCH_SINE_OVERLAY_CLIENT_H
