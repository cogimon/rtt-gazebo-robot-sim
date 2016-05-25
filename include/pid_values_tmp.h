#ifndef _PID_VALUES_TMP_
#define _PID_VALUES_TMP_

#include <gazebo/common/common.hh>

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

#endif
