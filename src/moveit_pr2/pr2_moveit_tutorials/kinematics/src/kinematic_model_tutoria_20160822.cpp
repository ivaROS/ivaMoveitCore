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

/*header files for ROS*/
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <unistd.h>
#include <math.h>


/*header files for socket*/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h> 
#include <string>

#include <iostream>
#include <fstream>

int META_command_thread();
std::string lindmod_pos_thread(double& x, double& y);

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
	///////////////////////////////////////////////////////////////////
	//                     Arm initialization                        //
	///////////////////////////////////////////////////////////////////

	
	double x_pos_from_lindmod;
	double y_pos_from_lindmod;
    std::string command = lindmod_pos_thread(x_pos_from_lindmod, y_pos_from_lindmod);

    std::cout<<"command"<<command<<std::endl;
    std::cout<<"x_pos_from_lindmod"<<x_pos_from_lindmod<<std::endl;
    std::cout<<"y_pos_from_lindmod"<<y_pos_from_lindmod<<std::endl;
    x_pos_from_lindmod += 0.25; 
    y_pos_from_lindmod -= 0.15;
    std::cout<<"x_pos_from_lindmod modified"<<x_pos_from_lindmod<<std::endl;
    std::cout<<"y_pos_from_lindmod modified"<<y_pos_from_lindmod<<std::endl;
 

//META_command_thread();

	// Setup current_joint_values for arm
	for (std::size_t i = 0; i < 7; ++i)
	{
		current_joint_values.push_back(0);
	}
	ros::init(argc, argv, "arm_kinematics");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Start joint_model_group for arm
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	/* 1. model_loader gets kinematic_model*/
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel(); 
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str()); 
	/* 2. kinematic_model gets kinematic_state*/
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model)); 
	kinematic_state->setToDefaultValues();
	/* 3. kinematic_model gets joint_model_group*/
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm"); 
	/* 4. joint_model_group gets joint_names*/
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames(); 

	// Subscribe Joint_State_Values
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);

	// Joint Limits
	/* setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.*/
	/* 1 .Set one joint in the right arm outside its joint limit */
	kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
	/* 2. Check whether any joint is outside its joint limits */
	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
	/* 3. Enforce the joint limits for this state and check again*/
	kinematic_state->enforceBounds();
	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

	//Publish Joint State Values
	/*setup publishers*/
	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_1/command", 1, true);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_2/command", 1, true);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_3/command", 1, true);
	ros::Publisher pub_4 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_4/command", 1, true);
	ros::Publisher pub_5 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_5/command", 1, true);
	ros::Publisher pub_6 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_6/command", 1, true);
	ros::Publisher pub_7 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_7/command", 1, true);
	ros::Publisher pub_8 = n.advertise<std_msgs::Float64>("/finalasm_position_controller_8/command", 1, true);
	/*setup value type for publishers*/ 
	std_msgs::Float64 val_1;
	std_msgs::Float64 val_2;
	std_msgs::Float64 val_3;
	std_msgs::Float64 val_4;
	std_msgs::Float64 val_5;
	std_msgs::Float64 val_6;
	std_msgs::Float64 val_7;
	std_msgs::Float64 val_8;
	/*initial joint values*/
	/*modified here for initial position!!*/
	val_1.data = -0.35;
	val_2.data = -1;
	val_3.data = 0;
	val_4.data = -1;
	val_5.data = 0;
	val_6.data = 0.35;
	val_7.data = 0;
	val_8.data = -0.2;
    /*publish values*/
	pub_1.publish(val_1);
	pub_2.publish(val_2);
	pub_3.publish(val_3);
	pub_4.publish(val_4);
	pub_5.publish(val_5);
	pub_6.publish(val_6);
	pub_7.publish(val_7);  
	pub_8.publish(val_8);


	///////////////////////////////////////////////////////////////////
	//                      Setup Parameters                         //
	///////////////////////////////////////////////////////////////////

	//Define and initialize objects
	Eigen::VectorXd initialJoints(7);
	Eigen::VectorXd initialPose(6);
	Eigen::Matrix4d initialPoseG;
	
	Eigen::VectorXd finalPose(6);
	Eigen::MatrixXd finalPoseG = Eigen::MatrixXd::Identity(4,4);
	
	Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,7);
	Eigen::MatrixXd jacPseudoInv;
	Eigen::MatrixXd jacobianNEW = Eigen::MatrixXd::Zero(5,7);

	Eigen::Vector3d v(0.0,0.0,0.0);
	Eigen::Vector3d w(0.0,0.0,0.0);
	Eigen::VectorXd twist(6);
	Eigen::VectorXd twistNEW = Eigen::VectorXd::Zero(5);

	Eigen::VectorXd newPose(6);
	Eigen::VectorXd newJoints;
	Eigen::Matrix4d relativePoseG;
	Eigen::Matrix3d relativeR;
	Eigen::Vector3d relativeP(0.0,0.0,0.0);

	Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
	Eigen::Matrix3d eyes = Eigen::MatrixXd::Identity(3,3);
	Eigen::Matrix3d W_head;

	Eigen::VectorXd diff(6);  
	double tau = 0.0;
	double traceR = 0.0;
	int poseInd = 0;
	bool returnFlag = false;

    
	sleep(7);
	kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
	const Eigen::Affine3d &temp = kinematic_state->getGlobalLinkTransform("link_8");
	Eigen::Matrix3d initialRot = temp.rotation();
	Eigen::Vector3d initialEuler;
	initialEuler << temp.rotation().eulerAngles(0,1,2);
	ROS_INFO_STREAM("initial position: \n" << temp.translation());
	ROS_INFO_STREAM("initial eulerAngles: \n" << initialEuler << "\n");
	Eigen::MatrixXd W = Eigen::MatrixXd::Identity(7,7);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5,5);

	W(0,0) = 1;
	W(1,1) = 1;
	W(2,2) = 1;
	W(3,3) = 1;
	W(4,4) = 1;
	W(5,5) = 1;
	W(6,6) = 10;


    ///////////////////////////////////////////////////////////////////
	//                       Loop for poses                          //
	///////////////////////////////////////////////////////////////////
/*
	double x_pos_from_lindmod;
	double y_pos_from_lindmod;
    std::string command = lindmod_pos_thread(x_pos_from_lindmod, y_pos_from_lindmod);

    std::cout<<"command"<<command<<std::endl;
    std::cout<<"x_pos_from_lindmod"<<x_pos_from_lindmod<<std::endl;
    std::cout<<"y_pos_from_lindmod"<<y_pos_from_lindmod<<std::endl;
    x_pos_from_lindmod += 0.25; 
    y_pos_from_lindmod -= 0.15;
    std::cout<<"x_pos_from_lindmod modified"<<x_pos_from_lindmod<<std::endl;
    std::cout<<"y_pos_from_lindmod modified"<<y_pos_from_lindmod<<std::endl;

*/
	while (true) {
		// Setting up poses to go
		poseInd++;
		switch (poseInd) {
			case 1:
				finalPose << 0.315, -0.085, 0.15, -0.199565, 1.09444, 1.57999;  //above cup position//0.40 and 0.15
				break;
				
			case 2:
				finalPose << 0.345, -0.085, 0.04, -0.199565, 1.09444, 1.57999;  //reach cup
				break;
		/*
			case 3:
				finalPose << 0.5, 0, 0.12, -0.199565, 1.09444, 1.57999;  //intermediate position
				break;
			case 4:
				finalPose << 0.45, -0.15, 0.12, -0.199565, 1.09444, 1.57999; //final position
				break;
		*/		
			default:
				returnFlag = true;
			}
		if (returnFlag) break;
		ROS_INFO_STREAM("target #" << poseInd << " translation: \n" << temp.translation().transpose() << "\n");

		// Constraints to stop
		double difference = 1000.0;
		double zDifference = 1000.0;
		double angDiff = 1000.0;
		double l1Norm = 1000.0;

		// Fixed orientation to be the same as the initial orientation
		finalPoseG.block(0,0,3,3) = initialRot;
		finalPoseG.block(0,3,3,1) = finalPose.block(0,0,3,1);


        ///////////////////////////////////////////////////////////////////
	    //                      Loop for Jacobian                        //
	    ///////////////////////////////////////////////////////////////////
        
		for(std::size_t i = 0; i < 30; ++i){
			// Set ending condition
			if(l1Norm < 0.03) {
				break;
			}
			
			sleep(1);
			// Get current joint values
			kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
			initialJoints << current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4], current_joint_values[5], current_joint_values[6];
			
			// Get current eef_state  
			const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link_8");
			initialPose << end_effector_state.translation(), end_effector_state.rotation().eulerAngles(0,1,2);
			initialPoseG = end_effector_state.matrix();

			// Get relativePoseG as spatial frame
			relativePoseG = finalPoseG*initialPoseG.inverse();
			relativeR = relativePoseG.block(0,0,3,3);
			relativeP = relativePoseG.block(0,3,3,1);
			traceR = relativeR(0,0) + relativeR(1,1) + relativeR(2,2);

			// Calculate v, w depending on tau
			tau = acos(0.5*(traceR - 1));
			if (tau<0.1)
			{
				w << 0.0,0.0,0.0;
				v << relativeP;
			}
			else
			{
				W_head = (relativeR - relativeR.transpose())/(2*sin(tau));
				w << W_head(2,1), W_head(0,2), W_head(1,0);
				v = ((eyes - relativeR)*W_head + w*w.transpose()*tau).inverse()*relativeP;
			}

			// Get twist
			twist << v, w;
			twist.block(0,0,3,1) = finalPose.block(0,0,3,1) - initialPose.block(0,0,3,1);

			// Get the Jacobian
			kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
			                           reference_point_position,
			                           jacobian);

			// Get inverse Jacobian
			jacobian = jacobian.block(0,0,6,7);
			jacobianNEW.block(0,0,5,7) = jacobian.block(0,0,5,7);
			jacPseudoInv = (W.inverse())*jacobianNEW.transpose()*((jacobianNEW*(W.inverse())*jacobianNEW.transpose() + 0.01*I).inverse());

			// Update twist
			twistNEW.block(0,0,5,1) = twist.block(0,0,5,1);
			ROS_INFO_STREAM("twist norm: " << twist.norm());

			// Calculate delta theta norm to avoid large change
			double angVelNorm = (jacPseudoInv*twistNEW).norm();
			ROS_INFO_STREAM("twist: " << twist.transpose());

			// Calculate new joints
			double gain = 1;
			if (twist.norm() < 0.1)
			{
				gain = 0.8;
			}
			if (angVelNorm >= 0.5)
			{
				newJoints = initialJoints + gain*(jacPseudoInv*twistNEW)/(angVelNorm*2)*0.6;
			}
			else
			{ 
				newJoints = initialJoints + gain*(jacPseudoInv*twistNEW)*0.6;
			}

			// Update joint values
			val_1.data = newJoints[0];
			val_2.data = newJoints[1];
			val_3.data = newJoints[2];
			val_4.data = newJoints[3];
			val_5.data = newJoints[4];
			val_6.data = newJoints[5];
			val_7.data = newJoints[6];

			pub_1.publish(val_1);
			pub_2.publish(val_2);
			pub_3.publish(val_3);
			pub_4.publish(val_4);
			pub_5.publish(val_5);
			pub_6.publish(val_6);
			pub_7.publish(val_7);  
			sleep(1);

			// Get current eef
			kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
			const Eigen::Affine3d &end_effector_state1 = kinematic_state->getGlobalLinkTransform("link_8");
			newPose << end_effector_state1.translation(), end_effector_state1.rotation().eulerAngles(0,1,2);

			// Compute diffs and norms
			diff = (finalPose - newPose).block(0,0,3,1);
			ROS_INFO_STREAM("diff: " << diff.transpose());
			if (diff(0)<0) diff(0) *= -1;
			if (diff(1)<0) diff(1) *= -1;
			if (diff(2)<0) diff(2) *= -1;
			ROS_INFO_STREAM("finalPose: " << finalPose.transpose());
			ROS_INFO_STREAM("newPose: " << newPose.transpose());
			l1Norm = std::max(diff(1),diff(0));
			l1Norm = std::max(l1Norm, diff(2));
			angDiff = (newPose.block(3,0,3,1) - initialEuler).norm();
			ROS_INFO_STREAM("ang diff: " << angDiff);
			zDifference = finalPose(2)-newPose(2);
			if (zDifference < 0) zDifference *= -1;
			ROS_INFO_STREAM("zDiff: " << zDifference);
			ROS_INFO_STREAM("zNew: " << newPose(2));
			ROS_INFO_STREAM("position l1_Norm: " << l1Norm);
			ROS_INFO_STREAM("Pose " << poseInd <<" loop "<< i << " done .." << '\n');
		}
		sleep(3);

        /*
		//grab when reach cup position
		if (poseInd == 2)
		{
			val_8.data = 0.4;
			pub_8.publish(val_8);
		}
		//release when reach final position
		if (poseInd == 4)
		{
			val_8.data = 0;
			pub_8.publish(val_8);
		}
		sleep(2);
		*/
	}

	
	ros::spin();
	return 0;
}



std::string lindmod_pos_thread(double& x, double& y){

	//since the compiler has not been upgraded(cannot do multiple threading), META command is read from commands.txt
    std::string command;
    


	int sockfd, new_fd, numbytes;
	struct sockaddr_in my_addr;
	struct sockaddr_in their_addr;
	uint sin_size;
	char buf[42];   //TCP socket
	std::string buf_str = "";
	if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 ){
		perror("socket");
		exit(1);
	}   //Initail, bind to port 2323 
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(2324);
	my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	bzero( &(my_addr.sin_zero), 8 );   //binding
	if ( bind(sockfd, (struct sockaddr*)&my_addr, sizeof(struct sockaddr)) == -1 ){
		perror("bind");
		exit(1);
	}   //Start listening
	if ( listen(sockfd, 10) == -1 ){
		perror("listen");
		exit(1);
	}   //Wait for connect!
	while(1){
		sin_size = sizeof(struct sockaddr_in);
		perror("server is run");
		if ( (new_fd = accept(sockfd, (struct sockaddr*)&their_addr, &sin_size)) == -1 ){
			perror("accept");
			exit(1);
		}   
        if ( !fork() ){
			//Receive
			while(1){
			if ( (numbytes = read(new_fd, buf, sizeof(buf))) == -1 ){
			  	perror("recv");
				exit(1);
			}
			printf("Receive %d bytes , ", numbytes);
			printf("%s\n", buf);
            buf_str = std::string(buf);
            std::cout<<"x = "<<buf_str.substr(13,5)<<std::endl;
            std::cout<<"y = "<<buf_str.substr(34,5)<<std::endl;
            x = boost::lexical_cast<double>(buf_str.substr(13,5));
            y = boost::lexical_cast<double>(buf_str.substr(34,5));
            std::cout<<"x(double)"<<x<<std::endl;
            std::cout<<"y(double)"<<y<<std::endl;

            std::ifstream myfile ("/home/fujenchu/Projects/ARM_project/socket/commands.txt");
            getline (myfile,command); 
            if(command != "") return command;
            myfile.close();
            //std::cout<<command<<std::endl;
            //std::string aa = "hold";
            //std::cout<< (aa == command)<<std::endl;
		    }
		}
	}
	close(sockfd);
	return 0;
}


int META_command_thread(){
	int sockfd, new_fd, numbytes;
	struct sockaddr_in my_addr;
	struct sockaddr_in their_addr;
	uint sin_size;
	char buf[4];   //TCP socket
	std::string buf_str = "";
	if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 ){
		perror("socket");
		exit(1);
	}   //Initail, bind to port 2323 
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(2326);
	my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	bzero( &(my_addr.sin_zero), 8 );   //binding
	if ( bind(sockfd, (struct sockaddr*)&my_addr, sizeof(struct sockaddr)) == -1 ){
		perror("bind");
		exit(1);
	}   //Start listening
	if ( listen(sockfd, 10) == -1 ){
		perror("listen");
		exit(1);
	}   //Wait for connect!
	while(1){
		sin_size = sizeof(struct sockaddr_in);
		perror("server is run for META_command");
		if ( (new_fd = accept(sockfd, (struct sockaddr*)&their_addr, &sin_size)) == -1 ){
			perror("accept");
			exit(1);
		}   
        if ( !fork() ){
			//Receive
			if ( (numbytes = read(new_fd, buf, sizeof(buf))) == -1 ){
			  	perror("recv");
				exit(1);
			}
			printf("Receive %d bytes , ", numbytes);
			printf("%s\n", buf);
            buf_str = std::string(buf);
            std::cout<<"command = "<<buf_str<<std::endl;
            //std::string command = "hold";
            //std::cout<<"command is hold"<<(command == buf_str)<<std::endl;
        }
		
	}
	close(sockfd);
	return 0;
}

