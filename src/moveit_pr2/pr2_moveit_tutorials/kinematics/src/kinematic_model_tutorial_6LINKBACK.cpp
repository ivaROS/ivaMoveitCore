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
/* Modified by: Fu-Jen Chu, Yushan Cai and Zhen Liu*/

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <math.h>
#include <unistd.h>

#include <iostream>
#include <fstream>


std::string getCommandFromMeta();

//current_joint_values stores the current joint values subscribed from joint_states topic
std::vector<double> current_joint_values;
void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg)
{
	current_joint_values.clear();
	for(std::size_t i = 0; i < msg->position.size(); ++i) {
		current_joint_values.push_back(msg->position[i]);
	}
}

int main(int argc, char **argv)
{


	/*****************************************************************
	*                         Arm initialization                     *
	*****************************************************************/
	// initialize joint angles for all 8 joints
	for (std::size_t i = 0; i < 6; ++i) { current_joint_values.push_back(0); }

    // ros initialization
	ros::init(argc, argv, "arm_kinematics");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// robot model & kinematic model loading
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

	/*****************************************************************
	*                  Subscribe Joint_State_Values                  *
	*****************************************************************/
	// subscribe joint values
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);

	// Check Joint Limits
	// set one joint in the right arm outside its joint limit 
	// kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
	// check whether any joint is outside its joint limits 
	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
	// enforce the joint limits for this state and check again
	kinematic_state->enforceBounds();
	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
	/*****************************************************************
	*                 Publish Joint State Values Setup               *
	*****************************************************************/
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


	/*****************************************************************
	*                   Publish Initial Joint Values                 *
	*****************************************************************/
	//initial joint values for raising the arm
	/*modified here for initial position!!*/
	val_1.data = 0;
	val_2.data = -1;
	val_3.data = 0;
	val_4.data = -0.7;
	val_5.data = 0;
	val_6.data = -1.5;
	val_7.data = 0;
	val_8.data = -0.5;

	pub_1.publish(val_1);
	pub_2.publish(val_2);
	pub_3.publish(val_3);
	pub_4.publish(val_4);
	pub_5.publish(val_5);
	pub_6.publish(val_6);
	pub_7.publish(val_7);  
	pub_8.publish(val_8);

	/*****************************************************************
	*                   Define and Initialize Objects                *
	*****************************************************************/
	Eigen::VectorXd initialJoints(6); // 7 by 1 vector 
	Eigen::VectorXd initialPose(6); // 6 by 1 vector for twist
	Eigen::Matrix4d initialPoseG; // 4 by 4 matrix for transformation
	
	Eigen::VectorXd finalLocation(3); 
	Eigen::MatrixXd finalPoseG = Eigen::MatrixXd::Identity(4,4);
	
	Eigen::MatrixXd jacobian_all = Eigen::MatrixXd::Zero(6,8); // 6 by 8 to catch Jacobian matrix from ROS
	Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,6); // 6 by 6 Jacobian matrix (6 joints)
	Eigen::MatrixXd jacPseudoInv; // 7 by 6 inverse Jacobian matrix (7 by 6 invJ* 6 by 1 twist = 7 by 1 new joint values )


	Eigen::Vector3d w(0.0,0.0,0.0);
	Eigen::Vector3d v(0.0,0.0,0.0);
	Eigen::VectorXd twist(6);


	Eigen::VectorXd currentLocation(3);
 	Eigen::VectorXd jointsVelocity;
	Eigen::VectorXd newJoints;
	Eigen::Matrix4d relativePoseG;
	Eigen::Matrix3d relativeR;
	Eigen::Vector3d relativeP(0.0,0.0,0.0);

	Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
	Eigen::Matrix3d eyes = Eigen::MatrixXd::Identity(3,3);
	Eigen::Matrix3d W_head;

	double diff = 1000.0;  
	double tau = 0.0;
	double traceR = 0.0;
	int poseInd = 0;
	bool returnFlag = false;

	int vnormScale = 0;
	int wnormScale = 0;

	sleep(7);
	kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
	const Eigen::Affine3d &temp = kinematic_state->getGlobalLinkTransform("link_7");
	Eigen::Matrix3d initialRot = temp.rotation();

	ROS_INFO_STREAM("initial position: \n" << temp.translation());

	/*****************************************************************
	*                       Reset META Command                       *
	*****************************************************************/
	std::ofstream myfileOut ("/home/fujenchu/Projects/ARM_project/socket/commands.txt", std::ios::trunc);
	myfileOut << "";
   	myfileOut.close();
    std::string command;
    bool checkFlag = false;
	/*****************************************************************
	*                       Termination parameters                   *
	*****************************************************************/
    double difference = 1000.0;
	double angDiff = 1000.0;
	bool processing = false;	

	/*****************************************************************
	*                    Start Inverse Jacobian Loop                 *
	*****************************************************************/
	while (true) {

        if(!processing){
			/*****************************************************************
			*                     Get Command from META                      *
			*****************************************************************/
			command =  getCommandFromMeta();
			if(command == ""){
				if(checkFlag == false) { std::cout<< "Handy is waiting for command!"<<std::endl; checkFlag = true;}
			}else{
				std::cout << "receive a command = " << command << std::endl; 
				checkFlag = false;
			}
        }// output can be (1) command, or (2) nothing



		if(command == "grab"){
			processing = true;
			poseInd++;
	    	switch (poseInd) {
				case 1:
			 	    finalPoseG.block(0,0,3,3) << 1,0,0,0,1,0,0,0,1;
					finalLocation << 0.40, 0.20, 0.3;
					finalPoseG.block(0,3,3,1) = finalLocation.block(0,0,3,1);
					break;

				case 2:
			 	    finalPoseG.block(0,0,3,3) << 1,0,0,0,1,0,0,0,1;
		      	    finalLocation << 0.40, 0.20, 0.1;
					finalPoseG.block(0,3,3,1) = finalLocation.block(0,0,3,1);
					break;

				case 3:
			 	    finalPoseG.block(0,0,3,3) << 1,0,0,0,1,0,0,0,1;
					finalLocation << 0.40, 0.20, 0.3;
					finalPoseG.block(0,3,3,1) = finalLocation.block(0,0,3,1);
					break;
				
				//case 4:
			  	// 	finalPoseG.block(0,0,3,3) = initialRot;
		      	//    finalLocation << 0.4, 0.2, 0.3;
				//	finalPoseG.block(0,3,3,1) = finalLocation.block(0,0,3,1);
				//	break;
			
				default:
					processing = false;
					poseInd = 0;
			}

		}else if(command == "hold"){
			processing = true;
			poseInd++;
	    	switch (poseInd) {
				case 1:
			 	    finalPoseG.block(0,0,3,3) = initialRot;
					finalLocation << 0.4, 0.2, 0;
					finalPoseG.block(0,3,3,1) = finalLocation.block(0,0,3,1);
					break;
				
				case 2:
			  	 	finalPoseG.block(0,0,3,3) = initialRot;
		      	    finalLocation << 0.6, 0, 0.1;
					finalPoseG.block(0,3,3,1) = finalLocation.block(0,0,3,1);
					break;
			
				default:
					processing = false;
					poseInd = 0;
			}


		}else{ 
			/*if command is "" then go back to the loop beginning*/
			continue; 

		}

	    std::cout << "poseInd = " << poseInd << std::endl;

		diff = 1000.0;
		size_t i = 0;
		while(diff > 0.08){	
			i++;


			//sleep(1);
			//get current joint values
			kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
			initialJoints << current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4], current_joint_values[5];
			//get current eef_state  
			const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link_7");
			initialPose << end_effector_state.translation(), end_effector_state.rotation().eulerAngles(0,1,2);
			initialPoseG = end_effector_state.matrix();

			/*****************************************************************
			*                 Joint values to inverse Jacobian               *
			*****************************************************************/
			// Get the Jacobian
			kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames()[5]), reference_point_position, jacobian_all);
			
			// Get inverse Jacobian of 6 links
			jacobian = jacobian_all.block(0,0,6,6);
			//ROS_INFO_STREAM("jacobian: " << jacobian);	

			Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(6,6);	
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
          	    for(size_t idx = 0; idx < 6; idx++){
            		double singularvalue = svd.singularValues()[idx];
            		if(singularvalue < 0.03) {S_inv(idx,idx) = 0.0;std::cout<<"singularity occurred!"<<std::endl;}
            		else S_inv(idx,idx) = 1/singularvalue;
           		}

			//U is 6 by 6
			//S is 6 by 7
			//V is 7 by 6
			Eigen::MatrixXd tempV = Eigen::MatrixXd::Ones(6,6);
			tempV.block(0,0,6,6) = svd.matrixV();

           	jacPseudoInv = tempV*S_inv*svd.matrixU().inverse();
            	
			/*****************************************************************
			*                    Relative Pose to Twist                      *
			*****************************************************************/
			// get relativePoseG as spatial frame
			relativePoseG = finalPoseG*initialPoseG.inverse();
			relativeR = relativePoseG.block(0,0,3,3);
			relativeP = relativePoseG.block(0,3,3,1);
			traceR = relativeR(0,0) + relativeR(1,1) + relativeR(2,2);

			// calculate v, w depending on tau
			tau = acos(0.5*(traceR - 1));
			if (tau < 0.1)
			{
				w << 0.0,0.0,0.0;
				v << relativeP;
			}
			else
			{
				W_head = (relativeR - relativeR.transpose())/(2*sin(tau));
				w << W_head(2,1), W_head(0,2), W_head(1,0);
				//ROS_INFO_STREAM("w: " << w.transpose());
				v = ((eyes - relativeR)*W_head + w*w.transpose()*tau).inverse()*relativeP;
				//ROS_INFO_STREAM("v: " << v.transpose());
			}

			// get twist
            vnormScale = 40;
            wnormScale = 10;


          	v = v/(vnormScale*v.norm());
            if(w.norm() != 0) w = w/(wnormScale*w.norm());
            else w << 0.0, 0.0, 0.0;
			twist << v, w;

			/*****************************************************************
			*                           Update Twist                         *
			*****************************************************************/
			ROS_INFO_STREAM("twist: " << twist.transpose());

            jointsVelocity = jacPseudoInv*twist;
			newJoints = initialJoints + jacPseudoInv*twist;

			//update joint values
			val_1.data = newJoints[0];
			val_2.data = newJoints[1];
			val_3.data = newJoints[2];
			val_4.data = newJoints[3];
			val_5.data = newJoints[4];
			val_6.data = newJoints[5];
			//val_7.data = newJoints[6];
			//val_8.data = newJoints[7];

			pub_1.publish(val_1);
			pub_2.publish(val_2);
			pub_3.publish(val_3);
			pub_4.publish(val_4);
			pub_5.publish(val_5);
			pub_6.publish(val_6);
			//pub_7.publish(val_7);  
			//pub_8.publish(val_8); 

			double maxLength = 0.0;
			double waitTime  = 0.0;
			for(size_t idx = 0; idx < 6; idx++){
			    if(std::abs(jointsVelocity[idx]) > maxLength) maxLength = std::abs(jointsVelocity[idx]);
			}
			waitTime = maxLength/0.5;
			waitTime = waitTime + 0.05; // second
			unsigned int microseconds = waitTime*1000000; // microsecond

			usleep(microseconds);
			

                        
			/*****************************************************************
			*                       Stopping Criteria                        *
			*****************************************************************/
			//get current eef
			kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
			const Eigen::Affine3d &end_effector_state1 = kinematic_state->getGlobalLinkTransform("link_7");
			currentLocation << end_effector_state1.translation();
                        //, end_effector_state1.rotation().eulerAngles(0,1,2);

			//compute diffs and norms
			diff = ((finalLocation - currentLocation).block(0,0,3,1)).norm();
			ROS_INFO_STREAM("Location difference: " << diff);
			ROS_INFO_STREAM("Location final  : " << finalLocation.transpose());
			ROS_INFO_STREAM("Location current: " << currentLocation.transpose());

			ROS_INFO_STREAM("Pose " << poseInd <<" loop "<< i << " done .." << '\n');



		}
        ROS_INFO_STREAM("done" << '\n');
		sleep(1);
		//grab when reach cup position
		if (poseInd == 2)
		{
			val_8.data = 0.4;
			pub_8.publish(val_8);
		}
		//release when reach final position
		if (poseInd == 4)
		{
			val_8.data = -1;
			pub_8.publish(val_8);
		}
		sleep(3);
	}
	ros::spin();
	return 0;
}


std::string getCommandFromMeta(){
	std::string command = "";
	std::ifstream myfileIn ("/home/fujenchu/Projects/ARM_project/socket/commands.txt");
    getline (myfileIn,command);

    if(command != ""){
	    std::ofstream myfileOut ("/home/fujenchu/Projects/ARM_project/socket/commands.txt", std::ios::trunc);
	    myfileOut << "";
   	    myfileOut.close();
    }
    myfileIn.close();
    return command;
}