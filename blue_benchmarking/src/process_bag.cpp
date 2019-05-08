#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>


#include <Eigen/Core>
#include <Eigen/Dense>


#include <rosbag/bag.h>
#include <iostream>
#include <fstream>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

template <typename TParam>
void getRequiredParam(ros::NodeHandle &nh, const std::string name, TParam &dest) {
  if(!nh.getParam(name, dest)) {
    ROS_FATAL("Could not find %s parameter in namespace %s", name.c_str(), nh.getNamespace().c_str());
    ros::shutdown();
    exit(1);
  }
}

KDL::Frame get_ee_pose(const sensor_msgs::JointState msg) {
  int nj = kdl_chain.getNrOfJoints();

  // Load joint positions into KDL
  KDL::JntArray joint_positions = KDL::JntArray(nj);
  for (int i = 0; i < nj; i++) {
    for (int j = 0; j < msg.name.size(); j++) {
      if (msg.name[j].compare(joint_names[i]) == 0) {
        joint_positions(i) = msg.position[j];
        break;
      }
      if (j == msg.name.size() - 1) {
        ROS_ERROR("ERRORRORORORORO");
      }
    }
  }
  // Build KDL solvers
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  // Compute cartesian position via KDL
  KDL::Frame cartesian_pose;
  if (fk_solver.JntToCart(joint_positions, cartesian_pose) < 0)
    ROS_ERROR("Forward kinematics failed");
  return cartesian_pose;
}

KDL::Frame get_ee_pose(const std_msgs::Float64MultiArray msg) {
  int nj = kdl_chain.getNrOfJoints();

  // Load joint positions into KDL
  KDL::JntArray joint_positions = KDL::JntArray(nj);
  for (int i = 0; i < nj; i++) {
      joint_positions(i) = msg.data[i];
  }
  // Build KDL solvers
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  // Compute cartesian position via KDL
  KDL::Frame cartesian_pose;
  if (fk_solver.JntToCart(joint_positions, cartesian_pose) < 0)
    ROS_ERROR("Forward kinematics failed");
  return cartesian_pose;
}



void process_bag(char* file_name) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("joint_states"));
  topics.push_back(std::string("/joint_states"));
  topics.push_back(std::string("/right_arm/blue_controllers/joint_torque_controller/command"));
  topics.push_back(std::string("/right_arm/nonlinear/command"));
  topics.push_back(std::string("/right_arm/error"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::ofstream ee_file;
  ee_file.open ("ee.csv");
  ee_file << "time,x,y,z,qx,qy,qz,qw,j0,j1,j2,j3,j4,j5,j6" << std::endl;

  std::ofstream cmd_file;
  cmd_file.open ("cmd.csv");
  cmd_file << "time,j0,j1,j2,j3,j4,j5,j6" << std::endl;

  std::ofstream err_file;
  err_file.open ("err.csv");
  err_file << "time,j0,j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11" << std::endl;

  std::ofstream traj_file;
  traj_file.open ("traj.csv");
  traj_file << "time,j0,j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11" << std::endl;


  foreach(rosbag::MessageInstance const m, view) {
      ros::Time msg_time = m.getTime();

      sensor_msgs::JointState::ConstPtr js = m.instantiate<sensor_msgs::JointState>();
      if (js != NULL) {
        // Load joint positions into KDL
        int nj = kdl_chain.getNrOfJoints();
        KDL::JntArray jpositions = KDL::JntArray(nj);
        KDL::JntArray jvelocities = KDL::JntArray(nj);
        for (int i = 0; i < nj; i++) {
          for (int j = 0; j < js->name.size(); j++) {
            if (js->name[j].compare(joint_names[i]) == 0) {
              jpositions(i) = js->position[j];
              jvelocities(i) = js->velocity[j];
              break;
            }
            if (j == js->name.size() - 1) {
              ROS_ERROR("ERRORRORORORORO");
            }
          }
        }

        ee_file << msg_time << ',';
        ee_file << jpositions(0) << ',' << jpositions(1) << ',' << jpositions(2) << ',' << jpositions(3) << ',' << jpositions(4) << ',' << jpositions(5) << ',' << jpositions(6) << ",";
        ee_file << jvelocities(0) << ',' << jvelocities(1) << ',' << jvelocities(2) << ',' << jvelocities(3) << ',' << jvelocities(4) << ',' << jvelocities(5) << ',' << jvelocities(6) << std::endl;
      }

      std_msgs::Float64MultiArray::ConstPtr j_cmd_ptr = m.instantiate<std_msgs::Float64MultiArray>();
      if (j_cmd_ptr != NULL && m.getTopic=="/right_arm/blue_controllers/joint_torque_controller/command") {
        std::vector<double> j_cmd = j_cmd_ptr->data;
        cmd_file << msg_time;
        cmd_file << ',' << j_cmd[0] << ',' << j_cmd[1] << ',' << j_cmd[2] << ',' << j_cmd[3] << ',' << j_cmd[4] << ',' << j_cmd[5] << ',' << j_cmd[6] << '\n';
      }

      std_msgs::Float64MultiArray::ConstPtr j_cmd_ptr = m.instantiate<std_msgs::Float64MultiArray>();
      if (j_cmd_ptr != NULL && m.getTopic=="/right_arm/error") {
        std::vector<double> j_cmd = j_cmd_ptr->data;
        err_file << msg_time;
        err_file << ',' << j_cmd[0] << ',' << j_cmd[1] << ',' << j_cmd[2] << ',' << j_cmd[3] << ',' << j_cmd[4] << ',' << j_cmd[5] << ',' << j_cmd[6] << ',' << j_cmd[7] << ',' << j_cmd[8] << ',' << j_cmd[9] << ',' << j_cmd[10] << ',' << j_cmd[11]<<'\n';
      }

      std_msgs::Float64MultiArray::ConstPtr j_cmd_ptr = m.instantiate<std_msgs::Float64MultiArray>();
      if (j_cmd_ptr != NULL && m.getTopic=="/right_arm/nonlinear/command") {
        std::vector<double> j_cmd = j_cmd_ptr->data;
        traj_file << msg_time;
        traj_file << ',' << j_cmd[0] << ',' << j_cmd[1] << ',' << j_cmd[2] << ',' << j_cmd[3] << ',' << j_cmd[4] << ',' << j_cmd[5] << ',' << j_cmd[6] << ',' << j_cmd[7] << ',' << j_cmd[8] << ',' << j_cmd[9] << ',' << j_cmd[10] << ',' << j_cmd[11]<<'\n';
      }


  }

  bag.close();
  ee_file.close();
  vive_file.close();
  cmd_file.close();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "inverse_kin_target");
  ros::NodeHandle node;
  if (argc != 2){
    ROS_ERROR("need bag file");
  }
  char* file_name = argv[1];
  ROS_ERROR("%s", file_name);


  process_bag(file_name);
  return 0;
}
