#ifndef _PID_VALUES_TMP_
#define _PID_VALUES_TMP_

#include <gazebo/common/common.hh>

class hardcoded_chains{
public:
    hardcoded_chains(){
        std::vector<std::string> joints;

        joints.push_back("lwr_arm_0_joint");
        joints.push_back("lwr_arm_1_joint");
        joints.push_back("lwr_arm_2_joint");
        joints.push_back("lwr_arm_3_joint");
        joints.push_back("lwr_arm_4_joint");
        joints.push_back("lwr_arm_5_joint");
        joints.push_back("lwr_arm_6_joint");
        map_chains_joints.insert(std::pair<std::string, std::vector<std::string>>("kukaarm", joints));
        joints.clear();

    }
    std::map<std::string, std::vector<std::string>> map_chains_joints;
};

class hardcoded_pids {
public:
	hardcoded_pids() {
		pids["lwr_arm_0_joint"] = gazebo::common::PID(1000.0, 0.1, 0.0);
		pids["lwr_arm_1_joint"] = gazebo::common::PID(1000.0, 0.1, 0.0);
		pids["lwr_arm_2_joint"] = gazebo::common::PID(1000.0, 0.1, 0.0);
		pids["lwr_arm_3_joint"] = gazebo::common::PID(3000.0, 0.1, 0.0);
		pids["lwr_arm_4_joint"] = gazebo::common::PID(5000.0, 0.1, 0.0);
		pids["lwr_arm_5_joint"] = gazebo::common::PID(3000.0, 0.1, 0.0);
		pids["lwr_arm_6_joint"] = gazebo::common::PID(3000.0, 0.1, 0.0);
	}

    std::map<std::string, gazebo::common::PID> pids;

};

class hardcoded_impedance {
public:
    hardcoded_impedance() {
        impedance["lwr_arm_0_joint"] = std::pair<double, double>(500., 20.);
        impedance["lwr_arm_1_joint"] = std::pair<double, double>(500., 20.);
        impedance["lwr_arm_2_joint"] = std::pair<double, double>(500., 20.);
        impedance["lwr_arm_3_joint"] = std::pair<double, double>(500., 20.);
        impedance["lwr_arm_4_joint"] = std::pair<double, double>(500., 20.);
        impedance["lwr_arm_5_joint"] = std::pair<double, double>(500., 20.);
        impedance["lwr_arm_6_joint"] = std::pair<double, double>(500., 20.);
    }

    /**
     * @brief impedance, std::pair<stiffness, damping>
     */
    std::map<std::string, std::pair<double, double>> impedance;

};

#endif
