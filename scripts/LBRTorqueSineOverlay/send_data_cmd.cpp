// #include <iiwa_ros.h>
#include <cmath>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <Eigen/StdVector>

#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

// #include <iiwa_ros/conversions.h>
#include <kdl/frames.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/Splines>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>
#include <string>

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>


ros::Publisher pub_pos_cmd;
#define KDATA_LOAD(h, p) h.getParam(#p, p)

class SplineFunction {

public:
  SplineFunction(Eigen::VectorXd const &x_vec,
                 Eigen::VectorXd const &y_vec)
    : x_min(x_vec.minCoeff()),
      x_max(x_vec.maxCoeff()),
      // Spline fitting here. X values are scaled down to [0, 1] for this.
      spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
                y_vec.transpose(),
                 // No more than cubic spline, but accept short vectors.

                std::min<int>(x_vec.rows() - 1, 3),
                scaled_values(x_vec)))
  { }

  double operator()(double x) const {
    // x values need to be scaled down in extraction as well.
    return spline_(scaled_value(x))(0);
  }

private:
  // Helpers to scale X values down to [0, 1]
  double scaled_value(double x) const {
    return (x - x_min) / (x_max - x_min);
  }

  Eigen::RowVectorXd scaled_values(Eigen::VectorXd const &x_vec) const {
    return x_vec.unaryExpr([this](double x) { return scaled_value(x); }).transpose();
  }

  double x_min;
  double x_max;

  // Spline of one-dimensional "points."
  Eigen::Spline<double, 1> spline_;
};


int readFile_nlines(std::string fileNm) {
    int n_data = 0;
    int x;
    std::ifstream inFile;
    std::string str;


    inFile.open(fileNm);
    if (!inFile) {
        std::cout << "Unable to open file";
        exit(1); // terminate with error
    }
    
    while (std::getline(inFile, str)) {
      n_data += 1;
    }
    
    inFile.close();
    
    return n_data;
}


void readFile(Eigen::MatrixXf &mat, std::string fileNm) {
    int n_data = 0;
    int x;
    std::ifstream inFile;
    std::string str;
    

    inFile.open(fileNm);
    if (!inFile) {
        std::cout << "Unable to open file";
        exit(1); // terminate with error
    }

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

    boost::char_separator<char> sep(" "); //Use space as the delimiter
    while (std::getline(inFile, str)) {
      std::vector<float> line;
      tokenizer tok(str, sep);
      // 
      std::transform( tok.begin(), tok.end(), std::back_inserter(line), 
                    &boost::lexical_cast<float,std::string> );
      int t = 0;
      for(std::vector<float>::iterator it = line.begin(); it != line.end(); ++it) {
        mat(t,n_data) = *it;
        // std::cout << line.size() << std::endl;
        t += 1;
      }
 
      n_data += 1;
      // std::copy(line.begin(), line.end(), std::ostream_iterator<float>(std::cout,"\n") ); //Print those for testing
    }

    inFile.close();
    // std::cout << "Sum = " << n_data << std::endl; 
    
}


int main (int argc, char **argv) {
  ros::init(argc, argv, "sendCmdData");
  ros::NodeHandle nh("~");

  std::string dir;

  nh.getParam("/dir_", dir);
  std::cout << "Data File Name: " << dir << std::endl;

  // To get the number of data points in the file.
  int n_data_qd, n_data_q;

  n_data_q = readFile_nlines((std::string)dir);

  Eigen::MatrixXf data_qd(8, n_data_qd);
  Eigen::MatrixXf data_q(8, n_data_q);

  // States to measure in the matrix
  Eigen::MatrixXf data_state_velocity(7,n_data_q);
  Eigen::MatrixXf data_state_position(7,n_data_q);
  Eigen::MatrixXf data_state_torque(7,n_data_q);
  iiwa_msgs::JointPosition command_joint_position;
  

  readFile(data_q, (std::string)dir);

  std::cout << "Number of data points : " << n_data_q << std::endl; 

  // pub_pos_cmd  = nh.advertise<iiwa_msgs::JointPosition>("/kuka/KUKACmdPosition", 1);
  pub_pos_cmd  = nh.advertise<trajectory_msgs::JointTrajectory>("/kuka/command_u_traj", 1);

  double update_rate = 50;
  ros::Rate loop_rate(update_rate);

  
  
  while(ros::ok()) {

    // Sleep for 10 seconds to make sure it gets there.

    int rem_points = n_data_q;
    int horizon = 0;

    for (int i=0; i < n_data_q; i++) {
      rem_points = n_data_q - i;

      horizon =  std::min<int>(rem_points, 20);                           
      
      if (horizon < 20) {
        horizon = rem_points;
      }

      // std::cout << "Horizon: " << horizon << std::endl;

      trajectory_msgs::JointTrajectory JointTraj;
      std::vector<trajectory_msgs::JointTrajectoryPoint> points_n(horizon);
      JointTraj.header.stamp = ros::Time::now();

      for (int j = 0;j < horizon; j++) {
        
        points_n[j].positions.resize(1);

        points_n[j].positions.push_back(data_q(1,i+j));
        points_n[j].positions.push_back(data_q(2,i+j));
        points_n[j].positions.push_back(data_q(3,i+j));
        points_n[j].positions.push_back(data_q(4,i+j));
        points_n[j].positions.push_back(data_q(5,i+j));
        points_n[j].positions.push_back(data_q(6,i+j));
        points_n[j].positions.push_back(data_q(7,i+j));
        points_n[j].time_from_start = ros::Duration(data_q(0,i+j));

        JointTraj.points.push_back(points_n[j]);

      }

      pub_pos_cmd.publish(JointTraj);

      // Sleep for satisfy the rate
      loop_rate.sleep();

      // Clear the vector
      points_n.clear();

      // Spin once
      ros::spinOnce();
    }

    ros::shutdown();
    
  }

  

}

