#ifndef _PID_VALUES_TMP_
#define _PID_VALUES_TMP_

#include <gazebo/common/common.hh>

class hardcoded_chains{
public:
    hardcoded_chains(){
        std::vector<std::string> joints;

        joints.push_back("WaistLat");
        joints.push_back("WaistSag");
        joints.push_back("WaistYaw");
        map_chains_joints.insert(std::pair<std::string, std::vector<std::string>>("torso", joints));
        joints.clear();

        joints.push_back("RHipSag");
        joints.push_back("RHipLat");
        joints.push_back("RHipYaw");
        joints.push_back("RKneeSag");
        joints.push_back("RAnkLat");
        joints.push_back("RAnkSag");
        map_chains_joints.insert(std::pair<std::string, std::vector<std::string>>("right_leg", joints));
        joints.clear();

        joints.push_back("LHipSag");
        joints.push_back("LHipLat");
        joints.push_back("LHipYaw");
        joints.push_back("LKneeSag");
        joints.push_back("LAnkLat");
        joints.push_back("LAnkSag");
        map_chains_joints.insert(std::pair<std::string, std::vector<std::string>>("left_leg", joints));
        joints.clear();

        joints.push_back("RShSag");
        joints.push_back("RShLat");
        joints.push_back("RShYaw");
        joints.push_back("RElbj");
        joints.push_back("RForearmPlate");
        joints.push_back("RWrj1");
        joints.push_back("RWrj2");
        map_chains_joints.insert(std::pair<std::string, std::vector<std::string>>("right_arm", joints));
        joints.clear();

        joints.push_back("LShSag");
        joints.push_back("LShLat");
        joints.push_back("LShYaw");
        joints.push_back("LElbj");
        joints.push_back("LForearmPlate");
        joints.push_back("LWrj1");
        joints.push_back("LWrj2");
        map_chains_joints.insert(std::pair<std::string, std::vector<std::string>>("left_arm", joints));
        joints.clear();

    }
    std::map<std::string, std::vector<std::string>> map_chains_joints;
};

class hardcoded_pids {
public:
    hardcoded_pids() {
        //Torso
        pids["WaistLat"] = gazebo::common::PID(1000.0, 0.1, 0.0);
        pids["WaistSag"] = gazebo::common::PID(1000.0, 0.1, 0.0);
        pids["WaistYaw"] = gazebo::common::PID(1000.0, 0.1, 0.0);

        //RLeg
        pids["RHipSag"] = gazebo::common::PID(3000.0, 0.1, 0.0);
        pids["RHipLat"] = gazebo::common::PID(5000.0, 0.1, 0.0);
        pids["RHipYaw"] = gazebo::common::PID(3000.0, 0.1, 0.0);
        pids["RKneeSag"] = gazebo::common::PID(3000.0, 0.1, 0.0);
        pids["RAnkLat"] = gazebo::common::PID(4000.0, 0.1, 0.0);
        pids["RAnkSag"] = gazebo::common::PID(3000.0, 0.1, 0.0);

        //LLeg
        pids["LHipSag"] = gazebo::common::PID(3000.0, 0.1, 0.0);
        pids["LHipLat"] = gazebo::common::PID(5000.0, 0.1, 0.0);
        pids["LHipYaw"] = gazebo::common::PID(3000.0, 0.1, 0.0);
        pids["LKneeSag"] = gazebo::common::PID(3000.0, 0.1, 0.0);
        pids["LAnkLat"] = gazebo::common::PID(4000.0, 0.1, 0.0);
        pids["LAnkSag"] = gazebo::common::PID(3000.0, 0.1, 0.0);

        //RArm
        pids["RShSag"] = gazebo::common::PID(1000.0, 0.1, 0.0);
        pids["RShLat"] = gazebo::common::PID(1000.0, 0.1, 0.0);
        pids["RShYaw"] = gazebo::common::PID(600.0, 0.1, 0.0);
        pids["RElbj"] = gazebo::common::PID(1000.0, 0.1, 0.0);
        pids["RForearmPlate"] = gazebo::common::PID(100.0, 0.1, 0.0);
        pids["RWrj1"] = gazebo::common::PID(100.0, 0.1, 0.0);
        pids["RWrj2"] = gazebo::common::PID(10.0, 0.01, 0.0);

        //LArm
        pids["LShSag"] = gazebo::common::PID(1000.0, 0.1, 0.0);
        pids["LShLat"] = gazebo::common::PID(1000.0, 0.1, 0.0);
        pids["LShYaw"] = gazebo::common::PID(600.0, 0.1, 0.0);
        pids["LElbj"] = gazebo::common::PID(1000.0, 0.1, 0.0);
        pids["LForearmPlate"] = gazebo::common::PID(100.0, 0.1, 0.0);
        pids["LWrj1"] = gazebo::common::PID(100.0, 0.1, 0.0);
        pids["LWrj2"] = gazebo::common::PID(10.0, 0.01, 0.0);
    }

    std::map<std::string, gazebo::common::PID> pids;

};

class hardcoded_impedance {
public:
    hardcoded_impedance() {
        //Torso
        impedance["WaistLat"] = std::pair<double, double>(800., 30.);
        impedance["WaistSag"] = std::pair<double, double>(800., 30.);
        impedance["WaistYaw"] = std::pair<double, double>(800., 30.);

        //RLeg
        impedance["RHipSag"] = std::pair<double, double>(1000., 100.);
        impedance["RHipLat"] = std::pair<double, double>(1000., 100.);
        impedance["RHipYaw"] = std::pair<double, double>(1000., 100.);
        impedance["RKneeSag"] = std::pair<double, double>(1000., 100.);
        impedance["RAnkLat"] = std::pair<double, double>(1000., 100.);
        impedance["RAnkSag"] = std::pair<double, double>(1000., 100.);

        //LLeg
        impedance["LHipSag"] = std::pair<double, double>(1000., 100.);
        impedance["LHipLat"] = std::pair<double, double>(1000., 100.);
        impedance["LHipYaw"] = std::pair<double, double>(1000., 100.);
        impedance["LKneeSag"] = std::pair<double, double>(1000., 100.);
        impedance["LAnkLat"] = std::pair<double, double>(1000., 100.);
        impedance["LAnkSag"] = std::pair<double, double>(1000., 100.);

        //RArm
        impedance["RShSag"] = std::pair<double, double>(500., 20.);
        impedance["RShLat"] = std::pair<double, double>(500., 20.);
        impedance["RShYaw"] = std::pair<double, double>(250., 10.);
        impedance["RElbj"] = std::pair<double, double>(500., 20.);
        impedance["RForearmPlate"] = std::pair<double, double>(100., 5.);
        impedance["RWrj1"] = std::pair<double, double>(100., 5.);
        impedance["RWrj2"] = std::pair<double, double>(100., 5.);

        //LArm
        impedance["LShSag"] = std::pair<double, double>(500., 20.);
        impedance["LShLat"] = std::pair<double, double>(500., 20.);
        impedance["LShYaw"] = std::pair<double, double>(250., 10.);
        impedance["LElbj"] = std::pair<double, double>(500., 20.);
        impedance["LForearmPlate"] = std::pair<double, double>(100., 5.);
        impedance["LWrj1"] = std::pair<double, double>(100., 5.);
        impedance["LWrj2"] = std::pair<double, double>(100., 5.);
    }

    /**
     * @brief impedance, std::pair<stiffness, damping>
     */
    std::map<std::string, std::pair<double, double>> impedance;

};

#endif
