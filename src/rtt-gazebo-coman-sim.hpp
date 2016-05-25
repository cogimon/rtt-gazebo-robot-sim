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
    bool setControlMode(const std::string& controlMode);

    bool initGazeboJointController();
    void setInitialPosition();
    bool setJointNamesAndIndices();

    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;

    RTT::SendHandle<gazebo::physics::ModelPtr(const std::string&, double)> get_model_handle;

    gazebo::physics::Joint_V gazebo_joints_;
    gazebo::physics::Link_V model_links_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> joint_scoped_names_;
    std::vector<int> joints_idx_;

    jointCtrl<rstrt::kinematics::JointAngles> jointPositionCtrl;
    jointCtrl<rstrt::dynamics::JointImpedance> jointImpedanceCtrl;
    jointCtrl<rstrt::dynamics::JointTorques> jointTorqueCtrl;

    jointFeedback<rstrt::kinematics::JointAngles> jointPositionFeedback;
    jointFeedback<rstrt::kinematics::JointVelocities> jointVelocityFeedback;
    jointFeedback<rstrt::dynamics::JointTorques> jointTorqueFeedback;

    std::string currentControlMode;

    gazebo::physics::JointControllerPtr gazebo_joint_ctrl;

private:
    bool is_configured;
};

}
#endif
