#ifndef _KINEMATIC_CHAIN_HPP_
#define _KINEMATIC_CHAIN_HPP_

#include <gazebo/gazebo.hh>
#include <vector>
#include <control_modes.h>
#include <boost/shared_ptr.hpp>

namespace cogimon {

class KinematicJoint {
public:
    KinematicJoint(gazebo::physics::JointPtr joint);
    KinematicJoint(gazebo::physics::JointPtr joint,
            std::vector<std::string> availableControlModes);
    ~KinematicJoint();
    void addAvailableControlMode(std::string mode);
    bool removeAvailableControlMode(std::string mode);
    std::vector<std::string> getAvailableControlModes();
    gazebo::physics::JointPtr getJoint();

protected:
    std::vector<std::string> availableControlModes;
    std::string currentControlMode;
    gazebo::physics::JointPtr joint;
};

typedef boost::shared_ptr<KinematicJoint> KinematicJointPtr;

class KinematicChain {
public:
    KinematicChain(std::string name);
    ~KinematicChain();
    bool switchControlMode(std::string mode);
    std::vector<std::string> getAvailableControlModes();
    void addJoint(KinematicJointPtr joint);
    bool removeJoint(KinematicJointPtr joint);
    std::string getActiveControlMode();
    KinematicJointPtr getJointByName(std::string name);

protected:
    std::string name;
    void updateAvailableControlModesFromJoints();
    // either we have the information here or we deduce it from the joints every time.
    std::vector<std::string> availableControlModes;
    std::string currentControlMode;
    std::vector<KinematicJointPtr> joints;
};

typedef boost::shared_ptr<KinematicChain> KinematicChainPtr;
}
#endif
