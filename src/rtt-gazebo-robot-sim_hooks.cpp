#include <rtt-gazebo-robot-sim.hpp>
#include <Eigen/Dense>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void robotSim::WorldUpdateBegin() {
    if (!is_configured && !isRunning())
        return;

#ifdef USE_INTROSPECTION
    if (useCallTraceIntrospection) {
		cts_worldUpdate.call_time = time_service->getNSecs();
		cts_worldUpdate.call_type = rstrt::monitoring::CallTraceSample::CALL_START_WITH_DURATION;
#endif

    std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;
    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->getCommand();

    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->sense();

    for(unsigned int i = 0; i < force_torque_sensors.size(); ++i)
        force_torque_sensors[i].sense();

    for(unsigned int i = 0; i < imu_sensors.size(); ++i)
        imu_sensors[i].sense();

#ifdef USE_INTROSPECTION
    } else {
        std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;
        for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
            it->second->getCommand();

        for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
            it->second->sense();

        for(unsigned int i = 0; i < force_torque_sensors.size(); ++i)
            force_torque_sensors[i].sense();

        for(unsigned int i = 0; i < imu_sensors.size(); ++i)
            imu_sensors[i].sense();
    }
#endif
}

void robotSim::WorldUpdateEnd() {
    if (!is_configured && !isRunning())
        return;

    std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;
    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->move();

#ifdef USE_INTROSPECTION
    if (useCallTraceIntrospection) {
        cts_worldUpdate.call_duration = time_service->getNSecs();
		uint_least64_t wmect_tmp = cts_worldUpdate.call_duration - cts_worldUpdate.call_time;
		if (wmect_tmp > getWMECT()) {
            setWMECT(wmect_tmp);
		}
        processCTS(cts_worldUpdate);
    }
#endif
}
