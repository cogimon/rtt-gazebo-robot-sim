#ifndef COMAN_SIM_HPP
#define COMAN_SIM_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <Eigen/Dense>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <thread>
#include <memory>

// RST-RT includes
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>

#include <control_modes.h>

namespace cogimon {

class robotSim: public RTT::TaskContext {
public:
    robotSim(std::string const& name);
    bool configureHook();
    void updateHook();
    void WorldUpdateBegin();
    void WorldUpdateEnd();
    virtual ~robotSim() {}

protected:
    bool getModel(const std::string& gazebo_comp_name,
            const std::string& model_name, double timeout_s = 20.0);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    void setControlMode(const std::string& controlMode);

    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;

    RTT::SendHandle<gazebo::physics::ModelPtr(const std::string&, double)> get_model_handle;

    gazebo::physics::Joint_V gazebo_joints_;
    gazebo::physics::Link_V model_links_;
    std::vector<std::string> joint_names_;
    std::vector<int> joints_idx_;

    RTT::InputPort<rstrt::kinematics::JointAngles> port_JointPositionCommand;
    RTT::InputPort<rstrt::dynamics::JointImpedance> port_JointImpedanceCommand;
    RTT::InputPort<rstrt::dynamics::JointTorques> port_JointTorqueCommand;

    RTT::FlowStatus jnt_trq_cmd_fs, jnt_pos_cmd_fs, jnt_imp_cmd_fs;

    RTT::OutputPort<rstrt::kinematics::JointAngles> port_JointPosition;
    RTT::OutputPort<rstrt::kinematics::JointVelocities> port_JointVelocity;
    RTT::OutputPort<rstrt::dynamics::JointTorques> port_JointTorque;

    rstrt::kinematics::JointAngles jnt_pos_cmd_, jnt_pos_;
    rstrt::dynamics::JointTorques jnt_trq_, jnt_trq_cmd_, jnt_trq_gazebo_cmd_;
    rstrt::kinematics::JointVelocities jnt_vel_, jnt_vel_cmd_;
    rstrt::dynamics::JointImpedance jnt_imp_, jnt_imp_cmd_;

    jointCtrlModes::ControlModes currentControlMode;

private:
    bool is_configured;
};

}
#endif
