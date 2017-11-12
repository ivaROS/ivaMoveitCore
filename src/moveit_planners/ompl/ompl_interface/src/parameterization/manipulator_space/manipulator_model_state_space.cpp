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

/* Author: Leo Keselman */

#include <moveit/ompl_interface/parameterization/manipulator_space/manipulator_model_state_space.h>
#include <ompl/base/spaces/ManipulatorStateSpace.h>
#include <ompl/base/spaces/ManipulatorState.h>
#include <moveit/profiler/profiler.h>

const std::string ompl_interface::ManipulatorModelStateSpace::PARAMETERIZATION_TYPE = "ManipulatorModel";

ompl_interface::ManipulatorModelStateSpace::ManipulatorModelStateSpace(const ModelBasedStateSpaceSpecification &spec) : ModelBasedStateSpace(spec)
{
    setName(getName() + "_" + PARAMETERIZATION_TYPE);
    manipulator_state_space_ = new ompl::base::ManipulatorStateSpace(variable_count_);
    //Load and connect to manipulator representation
    ManipulatorState* manState = new ManipulatorState();
    manState->Initialize("robot_description", spec.joint_model_group_->getName());
    manipulator_state_space_->setManipulatorStatePtr(manState);
}

ompl_interface::ManipulatorModelStateSpace::~ManipulatorModelStateSpace()
{
}

double ompl_interface::ManipulatorModelStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    //std::cout<<"Getting distance..."<<std::endl;
    return manipulator_state_space_->distance(state1->as<StateType>()->manipulator_state_, state2->as<StateType>()->manipulator_state_);
}

ompl::base::State* ompl_interface::ManipulatorModelStateSpace::allocState() const
{
    //std::cout<<"Allocating state..."<<std::endl;
    StateType* newState = new StateType();
    newState->manipulator_state_ = manipulator_state_space_->allocState();
    return newState;
}

void ompl_interface::ManipulatorModelStateSpace::freeState(ompl::base::State *state) const
{
    //std::cout<<"Freeing state..."<<std::endl;
    manipulator_state_space_->freeState(state->as<StateType>()->manipulator_state_);
    ModelBasedStateSpace::freeState(state);
}

void ompl_interface::ManipulatorModelStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{
    //std::cout<<"Copying State..."<<std::endl;
    //std::cout<<"Do we have a destination? "<<destination<<std::endl;
    //std::cout<<"Do we have a source? "<<source<<std::endl;
    manipulator_state_space_->copyState(destination->as<StateType>()->manipulator_state_, source->as<StateType>()->manipulator_state_);
    //std::cout<<"Done copying"<<std::endl;
}

void ompl_interface::ManipulatorModelStateSpace::sanityChecks() const
{
    ModelBasedStateSpace::sanityChecks(std::numeric_limits<double>::epsilon(), std::numeric_limits<float>::epsilon(), ~ompl::base::StateSpace::STATESPACE_TRIANGLE_INEQUALITY);
}

void ompl_interface::ManipulatorModelStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{
    state->as<StateType>()->clearKnownInformation();
    //std::cout<<"Interpolating... Step size: "<<t<<std::endl;
    //std::cout<<"From state: "<<std::endl;
    //manipulator_state_space_->printState(from->as<StateType>()->manipulator_state_, std::cout);
    manipulator_state_space_->interpolate(from->as<StateType>()->manipulator_state_, to->as<StateType>()->manipulator_state_, 
					  t, state->as<StateType>()->manipulator_state_); 
}

ompl::base::StateSamplerPtr ompl_interface::ManipulatorModelStateSpace::allocDefaultStateSampler() const
{
    class DefaultStateSampler : public ompl::base::StateSampler
    {
    public:
 
    DefaultStateSampler(const ompl::base::StateSpace *space,
    const robot_model::JointModelGroup *group,
    const robot_model::JointBoundsVector *joint_bounds)
    : ompl::base::StateSampler(space)
    , joint_model_group_(group)
    , joint_bounds_(joint_bounds)
    {
    }
 
    virtual void sampleUniform(ompl::base::State *state)
    {
       std::cout<<"Sampling random state uniformly"<<std::endl;
       std::vector<double> values(space_->getDimension());
       joint_model_group_->getVariableRandomPositions(moveit_rng_, &values[0], *joint_bounds_);
       for(unsigned int i = 0; i< space_->getDimension(); ++i)
       {
          state->as<StateType>()->manipulator_state_->as<ompl::base::ManipulatorStateSpace::StateType>()->setJoints(values[i],i);
       }
       state->as<StateType>()->setJointsComputed(true);
       state->as<StateType>()->setPoseComputed(false);
       state->as<StateType>()->clearKnownInformation();
       //std::cout<<"returning random state"<<std::endl;
       //space_->as<ompl::base::ManipulatorStateSpace>()->printState(state, std::cout);
    }
 
     virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
     {
       std::vector<double> values(space_->getDimension());
       std::vector<double> jointsNear;
       
       for(int i = 0; i < space_->getDimension(); i++)
       {
           jointsNear.push_back(near->as<StateType>()->manipulator_state_->as<ompl::base::ManipulatorStateSpace::StateType>()->getJoints(i));
       }
       
       //std::vector<double> jointsNear = space_->as<ompl::base::ManipulatorStateSpace>()->getJoints(near);
       //std::cout<<"sample nearby according to joint values"<<std::endl;
       joint_model_group_->getVariableRandomPositionsNearBy(moveit_rng_, &values[0], *joint_bounds_,  &jointsNear[0], distance);
       //std::cout<<"setJoints"<<std::endl;
       
       std::cout<<"get joint value of sampled nearby state: ";
       for(int i = 0; i < 8; i++)
       {
           std::cout<<values[i]<<std::endl;
       }
       
       for(unsigned int i = 0; i< space_->getDimension(); ++i)
       {
          state->as<StateType>()->manipulator_state_->as<ompl::base::ManipulatorStateSpace::StateType>()->setJoints(values[i],i);
       }
       
       state->as<StateType>()->setJointsComputed(true);
       
       state->as<StateType>()->setPoseComputed(false);
       state->as<StateType>()->clearKnownInformation();
     }
     
/*     virtual void sampleUniformNear_joint(ompl::base::State *state, std::vector<double> joint, const double distance)
     {
       std::cout<<"Sampling random state uniformNearly"<<std::endl;
       std::cout<<"set values[] space dimension is "<<space_->getDimension()<<std::endl;
       double values[space_->getDimension()];
       std::cout<<"joint_model_group_ getVariableRandomPositionsNearBy"<<std::endl;
       joint_model_group_->getVariableRandomPositionsNearBy(moveit_rng_, &values[0], *joint_bounds_,  &joint[0], distance);
       for(unsigned int i = 0; i< space_->getDimension(); ++i)
       {
          std::cout<<"setJoints"<<std::endl;
          state->as<StateType>()->manipulator_state_->as<ompl::base::ManipulatorStateSpace::StateType>()->setJoints(values[i],i);
       }
       std::cout<<"setJointsComputed"<<std::endl;
       state->as<StateType>()->setJointsComputed(true);
       std::cout<<"setPoseComputed"<<std::endl;
       state->as<StateType>()->setPoseComputed(false);
     }
*/ 
     virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
     {
       sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
     }
 
     protected:
 
     random_numbers::RandomNumberGenerator moveit_rng_;
     const robot_model::JointModelGroup *joint_model_group_;
     const robot_model::JointBoundsVector *joint_bounds_;
  };
 
      return ompl::base::StateSamplerPtr(static_cast<ompl::base::StateSampler*>
      (new DefaultStateSampler(this, spec_.joint_model_group_, &spec_.joint_bounds_)));
}

void ompl_interface::ManipulatorModelStateSpace::copyToOMPLState(ompl::base::State *state, const robot_state::RobotState &rstate) const
{
  //std::cout<<"About to copy state"<<std::endl;
  Eigen::VectorXd rstateJoints;
  rstate.copyJointGroupPositions(spec_.joint_model_group_, rstateJoints);
  //std::cout<<"Joints copied. Vector: "<<rstateJoints<<std::endl;
  manipulator_state_space_->setJoints(state->as<StateType>()->manipulator_state_, rstateJoints);
  //std::cout<<"State updated"<<std::endl;
  state->as<StateType>()->setJointsComputed(true);
  state->as<StateType>()->setPoseComputed(false);
  //std::cout<<"Start state: "<<std::endl;
  //manipulator_state_space_->printState(state->as<StateType>()->manipulator_state_, std::cout);
}

bool ompl_interface::ManipulatorModelStateSpace::satisfiesBounds(const ompl::base::State *state) const
{
     std::vector<double> values = manipulator_state_space_->getJoints(state->as<StateType>()->manipulator_state_);
     if (std::isnan(values[0]))
	return false;
     //std::cout<<"Testing this joint config: "<< manipulator_state_space_->getEigenJoints(state->as<StateType>()->manipulator_state_);
     //std::cout<<"Within bounds? "<< spec_.joint_model_group_->satisfiesPositionBounds(&values[0], spec_.joint_bounds_, std::numeric_limits<double>::epsilon())<<std::endl;
     if(state->as<StateType>()->jointsComputed())
         return spec_.joint_model_group_->satisfiesPositionBounds(&values[0], spec_.joint_bounds_, std::numeric_limits<double>::epsilon());
     else
	 return true;
	 //!!In the future, check if within workspace bounds
}

void ompl_interface::ManipulatorModelStateSpace::copyToRobotState(robot_state::RobotState& rstate, const ompl::base::State *state) const
{
    rstate.setJointGroupPositions(spec_.joint_model_group_, manipulator_state_space_->getEigenJoints(state->as<StateType>()->manipulator_state_));
    rstate.update();
}
