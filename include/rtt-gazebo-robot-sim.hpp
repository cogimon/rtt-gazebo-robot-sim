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


#include <control_modes.h>
#include <kinematic_chain.h>
#include <boost/shared_ptr.hpp>

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


    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;

    RTT::SendHandle<gazebo::physics::ModelPtr(const std::string&, double)> get_model_handle;

    gazebo::physics::Joint_V gazebo_joints_;
    gazebo::physics::Link_V model_links_;

    boost::shared_ptr<KinematicChain> whole_robot;

private:
    bool is_configured;
};

}
#endif
