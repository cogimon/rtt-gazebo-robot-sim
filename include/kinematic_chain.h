#ifndef _KINEMATIC_CHAIN_H_
#define _KINEMATIC_CHAIN_H_

// RST-RT includes
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/robot/JointState.hpp>

#include <control_modes.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <parser.h>

#ifdef USE_INTROSPECTION
#include <rtt-core-extensions/rtt-introspection-base.hpp>
#endif

using namespace rstrt::kinematics;
using namespace rstrt::dynamics;
using namespace rstrt::robot;


typedef cogimon::jointCtrl<JointAngles> position_ctrl;
typedef cogimon::jointCtrl<JointImpedance> impedance_ctrl;
typedef cogimon::jointCtrl<JointTorques> torque_ctrl;

typedef cogimon::jointFeedback<JointState> full_fbk;


class KinematicChain {
public:
    KinematicChain(const std::string& chain_name, const std::vector<std::string>& joint_names,
                   RTT::DataFlowInterface& ports, gazebo::physics::ModelPtr model
#ifdef USE_INTROSPECTION
                   ,cogimon::RTTIntrospectionBase* introBase
#endif
                   );
    ~KinematicChain(){}

    std::string getKinematicChainName();
    unsigned int getNumberOfDOFs();
    std::string getCurrentControlMode();
    std::vector<std::string> getJointNames();
    std::vector<std::string> getControllersAvailable();
    bool initKinematicChain(const cogimon::gains& gains_);
    bool resetKinematicChain();
    bool setControlMode(const std::string& controlMode);
    void sense();
    void getCommand();
    void move();
    std::string printKinematicChainInformation();

    bool setInitialJointConfiguration(const std::vector<double>& home);


    boost::shared_ptr<position_ctrl> position_controller;
    boost::shared_ptr<impedance_ctrl> impedance_controller;
    boost::shared_ptr<torque_ctrl> torque_controller;

    boost::shared_ptr<full_fbk> full_feedback;

private:
    std::string _kinematic_chain_name;
    std::vector<std::string> _controllers_name;
    unsigned int _number_of_dofs;
    RTT::DataFlowInterface& _ports;


    gazebo::physics::ModelPtr _model;
    std::string _current_control_mode;
    gazebo::physics::JointControllerPtr _gazebo_position_joint_controller;
    std::vector<std::string> _joint_names;
    std::map<std::string, std::string> _map_joint_name_scoped_name;

    std::vector<double> _initial_joints_configuration;

    bool setController(const std::string& controller_type);
    void setFeedBack();
    bool setJointNamesAndIndices();
    bool initGazeboJointController();
    std::vector<std::string> getJointScopedNames();
    void setInitialPosition(const bool use_actual_model_pose = true);
    void setInitialImpedance();

    boost::shared_ptr<cogimon::gains> _gains;
    
#ifdef USE_INTROSPECTION
    cogimon::RTTIntrospectionBase* _introBase;
#endif
};



#endif
