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

#ifndef MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_WORK_SPACE_MANIPULATOR_MODEL_STATE_SPACE_
#define MOVEIT_OMPL_INTERFACE_PARAMETERIZATION_WORK_SPACE_MANIPULATOR_MODEL_STATE_SPACE_

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <ompl/base/spaces/ManipulatorStateSpace.h>
namespace ompl_interface
{

class ManipulatorModelStateSpace : public ModelBasedStateSpace
{
public:

  static const std::string PARAMETERIZATION_TYPE;

  class StateType : public ModelBasedStateSpace::StateType
  {
  public:
    
    StateType() 
      : ModelBasedStateSpace::StateType()
    {
    }
    
    bool jointsComputed() const
    {
        return manipulator_state_->as<ompl::base::ManipulatorStateSpace::StateType>()->jointConfigKnown;
    }
    
    bool poseComputed() const
    {
        return manipulator_state_->as<ompl::base::ManipulatorStateSpace::StateType>()->eefPoseKnown;
    }
    
    void setJointsComputed(bool value)
    {
        manipulator_state_->as<ompl::base::ManipulatorStateSpace::StateType>()->jointConfigKnown = value;
    }

    void setPoseComputed(bool value)
    {
        manipulator_state_->as<ompl::base::ManipulatorStateSpace::StateType>()->eefPoseKnown = value;
    }
    
    ompl::base::State* manipulator_state_;
  };
  
  ManipulatorModelStateSpace(const ModelBasedStateSpaceSpecification &spec);
  virtual ~ManipulatorModelStateSpace();

  virtual const std::string getParameterizationType(void) const
  {
      return PARAMETERIZATION_TYPE;
  }
  
  virtual ompl::base::State* allocState() const;
  virtual void freeState(ompl::base::State *state) const;
  virtual void copyState(ompl::base::State *destination, const ompl::base::State *source) const;
  virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;
  virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;

  virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;
  
  virtual void copyToOMPLState(ompl::base::State *state, const robot_state::RobotState &rstate) const;
  virtual void sanityChecks() const;

  virtual bool satisfiesBounds(const ompl::base::State *state) const;
  void copyToRobotState(robot_state::RobotState& rstate, const ompl::base::State *state) const;

  ompl::base::ManipulatorStateSpace* manipulator_state_space_;

private:

};

}

#endif
