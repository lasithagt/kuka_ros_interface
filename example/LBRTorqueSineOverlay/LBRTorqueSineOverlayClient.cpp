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
#include <iiwa_ros.h>
#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <iiwa_ros/conversions.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <models.hpp>
#include <kdl/framevel.hpp>

//#include <fstream>
//#include <iostream>

#include "LBRTorqueSineOverlayClient.h"
#include <ros/ros.h>
#include <kdl/chainiksolverpos_lma.hpp>

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
   printf("LBRTorqueSineOverlayClient initialized:\n"
         "\tjoint mask: 0x%x\n"
         "\tfrequency (Hz): %f\n"
         "\tamplitude (Nm): %f\n",
         jointMask, freqHz, torqueAmplitude);
   
   for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){ _torques[i] = 0.0;}
   sub_torque = nh.subscribe("/iiwa/KUKACmdTorque", 1, &LBRTorqueSineOverlayClient::getKUKATorqueCmd, this);
   pub_torque = nh.advertise<iiwa_msgs::JointTorque>("/kuka/KUKAActualTorque", 1); 
   pub_position = nh.advertise<iiwa_msgs::JointPosition>("/kuka/KUKAJointPosition", 1); 

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


void LBRTorqueSineOverlayClient::publishState(double jointTorque[], double jointState[])
{
  iiwa_msgs::JointTorque kukaTorque;
  iiwa_msgs::JointPosition kukaPosition;

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


  pub_torque.publish(kukaTorque);
  pub_position.publish(kukaPosition);

}
//******************************************************************************
void LBRTorqueSineOverlayClient::command()
{
    // In command(), the joint values have to be sent. Which is done by calling
    // the base method.
    LBRClient::command();

    double jointPos[LBRState::NUMBER_OF_JOINTS];
    double jointPosDes[LBRState::NUMBER_OF_JOINTS];
    jointPosDes[0] = 90.0* M_PI/180; jointPosDes[1] = 60.0* M_PI/180; jointPosDes[2] = 0.0* M_PI/180; jointPosDes[3] = 60.0* M_PI/180; jointPosDes[4] = 0.0* M_PI/180; jointPosDes[5] = 0.0* M_PI/180; jointPosDes[6] = 0.0* M_PI/180;
    memcpy(jointPos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    double _offset = 20.0* M_PI/180;

    double jointTorque[LBRState::NUMBER_OF_JOINTS];
    memcpy(jointTorque, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    publishState(jointTorque, jointPos);
    Eigen::VectorXd u(7);
    u(0) = 0; u(1) = 0; u(2) = 0; u(3) = 0; u(4) = 0; u(5) = 0; u(6) = 0;

    // Check for correct ClientCommandMode.
    if (robotState().getClientCommandMode() == TORQUE)
    { 
       // calculate  offset
      double offset = _torqueAmpl * sin(_phi);
       
      _phi += _stepWidth;
      if (_phi >= 2 * M_PI) _phi -= 2 * M_PI;      

       for (int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
       {
          if (_jointMask & (1<<i))
          { 
              ROS_INFO_STREAM("torque command" << _torques[0] << std::endl);
              // _torques[i] = 0*(jointPosDes[i] - jointPos[i]); // offset;
              jointPos[i] = jointPos[i]; //_offset;
          }
        }

       // Set superposed joint torques.
      robotCommand().setTorque(_torques);
      robotCommand().setJointPosition(jointPos);

    }
}

// This function is to compute the torque input to the KUKA arm.
void LBRTorqueSineOverlayClient::getKUKATorqueCmd(const iiwa_msgs::JointTorque::ConstPtr& msg) {
  _torques[0] = msg->torque.a1;
  _torques[1] = msg->torque.a2;
  _torques[2] = msg->torque.a3;
  _torques[3] = msg->torque.a4;
  _torques[4] = msg->torque.a5;
  _torques[5] = msg->torque.a6;
  _torques[6] = msg->torque.a7;
}

