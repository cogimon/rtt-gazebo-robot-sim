#include <rtt-gazebo-robot-sim.hpp>
#include <Eigen/Dense>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void robotSim::WorldUpdateBegin() {
    if (!is_configured && !isRunning())
        return;

    whole_robot->getCommand();

    whole_robot->sense();
}

void robotSim::WorldUpdateEnd() {
    if (!is_configured && !isRunning())
        return;

   whole_robot->move();
}
