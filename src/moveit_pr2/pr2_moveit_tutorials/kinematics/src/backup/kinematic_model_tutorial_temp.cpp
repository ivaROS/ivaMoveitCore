/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <math.h>

std::vector<double> current_joint_values;//current_joint_values stores the current joint values subscribed from /joint_states topic
void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg)
{
  current_joint_values.clear();
  for(std::size_t i = 0; i < msg->position.size(); ++i)
    current_joint_values.push_back(msg->position[i]);
}


int main(int argc, char **argv)
{
  for(std::size_t i = 0; i < 7; ++i)
    current_joint_values.push_back(0);

  ros::init (argc, argv, "arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL

  // Start
  // ^^^^^
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

  // Subscribe Joint_State_Values
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);


  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the right arm outside its joint limit */
  kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));


  //Publish Joint State Values
  //^^^^^^^^^^^^^^^^^^^^^^^^^^
  ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_1/command", 1, true);
  ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_2/command", 1, true);
  ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_3/command", 1, true);
  ros::Publisher pub_4 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_4/command", 1, true);
  ros::Publisher pub_5 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_5/command", 1, true);
  ros::Publisher pub_6 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_6/command", 1, true);
  ros::Publisher pub_7 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_7/command", 1, true);
  ros::Publisher pub_8 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_8/command", 1, true);
  std_msgs::Float64 val_1;
  std_msgs::Float64 val_2;
  std_msgs::Float64 val_3;
  std_msgs::Float64 val_4;
  std_msgs::Float64 val_5;
  std_msgs::Float64 val_6;
  std_msgs::Float64 val_7;
  std_msgs::Float64 val_8;

  //case 4 strange 
  val_1.data = -1.0;
  val_2.data = -0.4;
  val_3.data = -0.3;
  val_4.data = 1.7;
  val_5.data = 1.3;
  val_6.data = 1.2;
  val_7.data = -1.2;
  val_8.data = 0.5;

/*
  //case 3
  val_1.data = 1.0;
  val_2.data = 1.4;
  val_3.data = -0.3;
  val_4.data = 1.7;
  val_5.data = -1.3;
  val_6.data = 1.2;
  val_7.data = -1.2;
  val_8.data = 0.5;
*/
/*
  //case 2
  val_1.data = 1.0;
  val_2.data = -1.0;
  val_3.data = 1.3;
  val_4.data = 1.7;
  val_5.data = 1.3;
  val_6.data = -1.2;
  val_7.data = 1.2;
  val_8.data = 1.0;
*/ 
 /* 
  //case 1
  val_1.data = 0.9;
  val_2.data = 0.3;
  val_3.data = -1.4;
  val_4.data = 0.5;
  val_5.data = -1.2;
  val_6.data = 1.3;
  val_7.data = -1.3;
  val_8.data = -1.0;
*/
  pub_1.publish(val_1);
  pub_2.publish(val_2);
  pub_3.publish(val_3);
  pub_4.publish(val_4);
  pub_5.publish(val_5);
  pub_6.publish(val_6);
  pub_7.publish(val_7);  
  pub_8.publish(val_8);

  // should implement a function to transform final jointstates to finalpose (maybe a fake kinematic_state object)
  // define final pose
  Eigen::VectorXd finalpose_q(7);
  finalpose_q << 0.7, -0.02, 0.05, -0.5, -0.5, 0.5, -0.5;
  Eigen::VectorXd finalPose(6);
  finalPose << 0.7, -0.02, 0.05, 1.57, -0.01, -1.62;
  Eigen::MatrixXd finalPoseG(4,4);
  finalPoseG << -0.0840192, 0.996191,   -0.0233353 , 0.708261,
                0.00446867, -0.0230411, -0.999725  , -0.026399,
                -0.996454,  -0.0841004, -0.00251575, 0.0388889,
                0.0,        0.0,        0.0,         1.0;

  Eigen::VectorXd initialJoints(8);
  Eigen::VectorXd initialPose(6);
  Eigen::MatrixXd initialPoseG(4,4);
  Eigen::MatrixXd relativePoseG(4,4);

  Eigen::VectorXd newPose(6);
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd jacobianXYZ;
  Eigen::MatrixXd jacPseudoInv;
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd relativeR(3,3);
  Eigen::Vector3d relativeP(0.0,0.0,0.0);
  Eigen::MatrixXd W_head(3,3);
  Eigen::Vector3d w(0.0,0.0,0.0);
  Eigen::Vector3d v(0.0,0.0,0.0);
  Eigen::MatrixXd eyes(3,3);
  eyes = Eigen::MatrixXd::Identity(3,3);
  Eigen::VectorXd twist(6);
  
  Eigen::VectorXd newJoints;
  double tau = 0.0;
  double traceR = 0.0;
  //Eigen::VectorXd jointStateVelocity;

  double difference = 1000;
  sleep(5);
  for(std::size_t i = 0; i < 60; ++i){
    if(difference < 0.05) continue;
    sleep(1);
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    initialJoints << current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4], current_joint_values[5], current_joint_values[6], current_joint_values[7];
    
    //get current eef_state  
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link_9");
    initialPose << end_effector_state.translation(), end_effector_state.rotation().eulerAngles(0,1,2);
    initialPoseG = end_effector_state.matrix();

    ///////////////////////////////////////////////////////////
    //                                                       //
    // given initialPoseG and finalPoseG, find twist v and w //
    //                                                       //
    ///////////////////////////////////////////////////////////

    // get relativePoseG as spatial frame
    relativePoseG = finalPoseG*initialPoseG.inverse();
    relativeR = relativePoseG.block(0,0,3,3);
    relativeP = relativePoseG.block(0,3,3,1);
    traceR = relativeR(0,0) + relativeR(1,1) + relativeR(2,2);

    // calculate v, w depending on tau
    tau = acos(0.5*(traceR - 1));
    std::cout<<"tau is : "<<tau<<std::endl;
    if(tau<0.1){     
      std::cout<<"estimated tau!!!!!!!!!!!!!!!!!!!!"<< std::endl;
      w << 0.0,0.0,0.0;
      v << relativeP;
      v << (finalPoseG.block(0,3,3,1) - initialPoseG.block(0,3,3,1));
    }else{
      std::cout<<"true tau~~~~~~~~~~~~~~~~~~~~~~~~~"<< std::endl;
      W_head = (relativeR - relativeR.transpose())/(2*sin(tau));
      w << W_head(2,1), W_head(0,2), W_head(1,0);
      v = ((eyes - relativeR)*W_head + w*w.transpose()*tau).inverse()*relativeP;
    }

    // get twist
    twist << v, w;

    ///////////////////////////////////////////////////////////
    //                                                       //
    //           get Jacobian and inverse Jacobain           //
    //                                                       //
    ///////////////////////////////////////////////////////////

    // Get the Jacobian
    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position,
                               jacobian);

    // Get inverse Jacobian
    jacPseudoInv = jacobian.transpose()*((jacobian*jacobian.transpose()).inverse());

    if(tau<0.1){
      jacobianXYZ = jacobian.block(0,0,3,8);
      jacPseudoInv.block(0,0,8,3) = jacobianXYZ.transpose()*((jacobianXYZ*jacobianXYZ.transpose()).inverse());
    }
    //ROS_INFO_STREAM("jacPseudoInv: " <<'\n'<< jacPseudoInv);
    //ROS_INFO_STREAM("twist: " <<'\n'<< twist);
    ///////////////////////////////////////////////////////////
    //                                                       //
    //      calculate delta theta and update new joints      //
    //                                                       //
    ///////////////////////////////////////////////////////////
   
    // calculate delta theta norm to avoid large change    
    double angVelNorm = (jacPseudoInv*twist).norm();
    ROS_INFO_STREAM("(angVelNorm): " <<'\n'<< (angVelNorm));

    if(angVelNorm >= 0.5){
      newJoints = initialJoints + (jacPseudoInv*twist)/(angVelNorm*2);
      std::cout<<"~~~~~~~~normalized~~~~~~~~~`"<<std::endl;
    }else{ 
      newJoints = initialJoints + (jacPseudoInv*twist);
    }
    ROS_INFO_STREAM("original delta theta: " <<'\n'<< jacPseudoInv*twist);
    //ROS_INFO_STREAM("normalized delta theta: " <<'\n'<< (jacPseudoInv*twist)/(angVelNorm*2));
    //ROS_INFO_STREAM("normalized delta theta: " <<'\n'<< (finalPoseG.block(0,3,3,1) - initialPoseG.block(0,3,3,1)).norm());

    val_1.data = newJoints[0];
    val_2.data = newJoints[1];
    val_3.data = newJoints[2];
    val_4.data = newJoints[3];
    val_5.data = newJoints[4];
    val_6.data = newJoints[5];
    val_7.data = newJoints[6];
    val_8.data = newJoints[7];

    pub_1.publish(val_1);
    pub_2.publish(val_2);
    pub_3.publish(val_3);
    pub_4.publish(val_4);
    pub_5.publish(val_5);
    pub_6.publish(val_6);
    pub_7.publish(val_7);  
    pub_8.publish(val_8);

    sleep(1);
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    const Eigen::Affine3d &end_effector_state1 = kinematic_state->getGlobalLinkTransform("link_9");
    newPose << end_effector_state1.translation(), end_effector_state1.rotation().eulerAngles(0,1,2);
    ROS_INFO_STREAM("dot product: " << (newPose-initialPose).normalized().dot((finalPose-initialPose).normalized()));
    difference = (finalPose-newPose).block(0,0,3,1).norm();
    ROS_INFO_STREAM("difference: " << (finalPose-newPose).block(0,0,3,1).norm());

    std::cout<<"loop "<< i << " done .."<<std::endl<<std::endl;
  }
  
  ros::spin();
  //ros::shutdown();
  return 0;
}
