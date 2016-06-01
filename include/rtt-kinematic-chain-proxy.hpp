#ifndef RTT_KINEMATIC_CHAIN_PROXY_HPP
#define RTT_KINEMATIC_CHAIN_PROXY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>
#include <rtt/Operation.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <Eigen/Dense>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <string>
#include <thread>
#include <memory>

#include <control_modes.h>
#include <kinematic_chain.h>

// TODO data-types need a common super type
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointAccelerations.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>

namespace cogimon {

template<typename DERIVED>
class KinematicChainProxy: public RTT::TaskContext {
public:
	KinematicChainProxy(const std::string &name) :
			TaskContext(name), is_kinChain_set(false), is_external_port_set(
					false), portToSendFeedbackTo("feedback"), portToSplitFrom(
					name + "_port"), portToGetFeedbackFrom("sim_fb_port"), toBeSplitted_flow(RTT::NoData) {
//		BOOST_STATIC_ASSERT((boost::is_base_of<rci::JointValues, DERIVED>::value));
	}

	void setPortReference(RTT::DataFlowInterface* ports) {
		_ports = ports;
		is_external_port_set = true;
	}

	bool configureHook() {
		if (!is_kinChain_set) {
			RTT::log(RTT::Warning)
					<< "Kinematic chain needs to be set first! Use setKinematicChain(boost::shared_ptr<KinematicChain> chain)"
					<< RTT::endlog();
			return false;
		}

		return true;
	}

	void updateHook() {
		if (portToSplitFrom.connected()) {
			toBeSplitted_flow = portToSplitFrom.readNewest(dataToBeReceived);
			if (toBeSplitted_flow == RTT::NewData) {
				for (int i = 0; i < dataToBeReceived.size(); i++) {
					// check the lookup map
					dataToBePublished[lookupMap[i]](i) = dataToBeReceived(i);
				}
				for (int i = 0; i < portsToSplitTo.size(); i++) {
					if (portsToSplitTo[i]->connected()) {
						portsToSplitTo[i]->write(dataToBePublished[i]);
					}
				}
			}
		}
	}

	void setKinematicChain(boost::shared_ptr<KinematicChain> chain) {
		if (chain) {
			this->chain = chain;
			is_kinChain_set = true;
		} else {
			RTT::log(RTT::Warning)
					<< "Could not set kinematic chain. Pointer seems to be empty..."
					<< RTT::endlog();
		}
	}

	bool loadKinematicChainFromFile(std::string file) {
		// TODO check file and parse it properly!

		// TODO count amount of different components involved
		// set data chunk size accordingly
		componentNames.resize(2);
		dataToBePublished.resize(2); // TODO + set dummy value
		// TODO set dataToBeReceived
		// TODO set lookupMap[0] = 0;

		portToSendFeedbackTo.setDataSample(dataToBeReceived);

		if (is_external_port_set) {
			_ports->addPort(portToSendFeedbackTo).doc(
					"Output for " + portToSendFeedbackTo.getName());
			_ports->addPort(portToSplitFrom).doc(
					"Input for " + portToSplitFrom.getName());
			_ports->addPort(portToGetFeedbackFrom).doc(
					"Input for " + portToGetFeedbackFrom.getName());
		} else {
			this->ports()->addPort(portToSendFeedbackTo).doc(
					"Output for " + portToSendFeedbackTo.getName());
			this->ports()->addPort(portToSplitFrom).doc(
					"Input for " + portToSplitFrom.getName());
			this->ports()->addPort(portToGetFeedbackFrom).doc(
					"Input for " + portToGetFeedbackFrom.getName());
		}

		// iterate through chunks which have the same size as output ports
		for (int i = 0; i < componentNames.size(); i++) {
			// create the output ports
			boost::shared_ptr<RTT::OutputPort<DERIVED> > tmpPort(
					new RTT::OutputPort<DERIVED>(
							componentNames[i]
									+ boost::lexical_cast<std::string>(i)));
			tmpPort->setDataSample(dataToBePublished[i]);
			if (is_external_port_set) {
				_ports->addPort(*tmpPort).doc(
						"Output for " + tmpPort->getName());
			} else {
				this->ports()->addPort(*tmpPort).doc(
						"Output for " + tmpPort->getName());
			}
			portsToSplitTo.push_back(tmpPort);
		}

		is_kinChain_set = true;
		return true;
	}
	virtual ~KinematicChainProxy() {
	}

protected:
	boost::shared_ptr<KinematicChain> chain;

	// ports to which the data will be split and published to.
	std::vector<boost::shared_ptr<RTT::OutputPort<DERIVED> > > portsToSplitTo;
	// input port for the command which will be splitted according to the kinematic chains.
	RTT::InputPort<DERIVED> portToSplitFrom;
	// port to send the feedback vector to the controller-side
	RTT::OutputPort<DERIVED> portToSendFeedbackTo;
	// input port to receive the feedback from the Gazebo-side
	RTT::InputPort<DERIVED> portToGetFeedbackFrom;

private:
	std::map<int, int> lookupMap;

	std::vector<DERIVED> dataToBePublished;
	DERIVED dataToBeReceived;
	std::vector<std::string> componentNames;
	bool is_kinChain_set;
	bool is_external_port_set;

	RTT::DataFlowInterface* _ports;

	RTT::FlowStatus toBeSplitted_flow;
};

}
#endif
