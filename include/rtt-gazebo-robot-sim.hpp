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
    bool getModel(const std::string& model_name);
    void gazeboUpdateHook(gazebo::physics::ModelPtr model);
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
    bool setControlMode(const std::string& kinematic_chain, const std::string& controlMode);
    std::vector<std::string> getKinematiChains();
    std::string getControlMode(const std::string& kinematic_chain);
    std::vector<std::string> getControlAvailableMode(const std::string& kinematic_chain);
    std::string printKinematicChainInformation(const std::string& kinematic_chain);


    /**
     * Provides the joint name to index mapping for other components to retrieve.
     * If there isn't such an port (portName) existing, or used in an kinematic chain,
     * the call will return an empty map. Otherwise it will contain the mapping.
     */
    std::map<std::string, int> getJointMappingForPort(std::string portName);

    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;

    RTT::SendHandle<gazebo::physics::ModelPtr(const std::string&, double)> get_model_handle;

    gazebo::physics::Joint_V gazebo_joints_;
    gazebo::physics::Link_V model_links_;

    std::map<std::string, boost::shared_ptr<KinematicChain>> kinematic_chains;

private:
    bool is_configured;
};

}
#endif
