#ifndef _KINEMATIC_CHAIN_H_
#define _KINEMATIC_CHAIN_H_

// RST-RT includes
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>

#include <control_modes.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

using namespace rstrt::kinematics;
using namespace rstrt::dynamics;


typedef cogimon::jointCtrl<JointAngles> position_ctrl;
typedef cogimon::jointCtrl<JointImpedance> impedance_ctrl;
typedef cogimon::jointCtrl<JointTorques> torque_ctrl;

typedef cogimon::jointFeedback<JointAngles> position_fbk;
typedef cogimon::jointFeedback<JointVelocities> velocity_fbk;
typedef cogimon::jointFeedback<JointTorques> torque_fbk;


class KinematicChain {
public:
    KinematicChain(const std::string& chain_name, RTT::DataFlowInterface& ports, gazebo::physics::ModelPtr model);
    ~KinematicChain(){}

    std::string getKinematicChainName();
    unsigned int getNumberOfDOFs();
    std::string getCurrentControlMode();
    std::vector<std::string> getJointNames();
    std::vector<unsigned int> getJointIndices();
    std::vector<std::string> getControllersAvailable();
    bool initKinematicChain();
    bool setControlMode(const std::string& controlMode);
    void sense();
    void getCommand();
    void move();


    boost::shared_ptr<position_ctrl> position_controller;
    boost::shared_ptr<impedance_ctrl> impedance_controller;
    boost::shared_ptr<torque_ctrl> torque_controller;

    boost::shared_ptr<position_fbk> position_feedback;
    boost::shared_ptr<velocity_fbk> velocity_feedback;
    boost::shared_ptr<torque_fbk> torque_feedback;

private:
    std::string _kinematic_chain_name;
    std::vector<std::string> _controllers_name;
    unsigned int _number_of_dofs;
    RTT::DataFlowInterface& _ports;
    gazebo::physics::ModelPtr _model;
    std::string _current_control_mode;
    gazebo::physics::JointControllerPtr _gazebo_position_joint_controller;
    std::map<std::string, unsigned int> _map_joint_name_indices;
    std::map<std::string, std::string> _map_joint_name_scoped_name;

    bool setController(const std::string& controller_type);
    bool setFeedBack(const std::string& feedback_type);
    bool setJointNamesAndIndices();
    bool initGazeboJointController();
    std::vector<std::string> getJointScopedNames();
    void setInitialPosition();



};



#endif
