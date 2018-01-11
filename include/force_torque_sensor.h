#ifndef _FORCE_TORQUE_SENSOR_H_
#define _FORCE_TORQUE_SENSOR_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <XBotCoreModel.h>
#include <rtt/Port.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <gazebo/sensors/ForceTorqueSensor.hh>

#ifdef USE_INTROSPECTION
#include <rtt-core-extensions0.2/rtt-core-extensions/rtt-introspection-base.hpp>
//#include <rtt-core-extensions/rtt-introspection-base.hpp>
#endif

template <class T> class sensorFeedback {
public:
    sensorFeedback() {
    }

    ~sensorFeedback() {

    }

    T sensor_feedback;
    RTT::OutputPort<T> orocos_port;

};

typedef sensorFeedback<rstrt::dynamics::Wrench> wrench;

/**
 * @brief The force_torque_sensor class considers the force/torque sensors specified in the urdf
 * file and broadcasts the data in the OROCOS framework. Normally, the force/torque sensor is
 * specified wrt a joint of the model, then an offset is added to get the force/torque measurement
 * located in the real position of the sensor. For this reason in the model is specified also
 * another link (frame) that represent the sensor frame.
 */
class force_torque_sensor{
public:
    /**
     * @brief force_torque_sensor
     * @param joint_srdf is the joint name specified in the srdf under the group "force_torque_sensors"
     * @param model
     */
    force_torque_sensor(const std::string& joint_srdf, gazebo::physics::ModelPtr gazebo_model,
                        boost::shared_ptr<urdf::ModelInterface const> urdf_model,
                        gazebo::sensors::Sensor_V sensors,
                        RTT::DataFlowInterface& ports
#ifdef USE_INTROSPECTION
                        ,cogimon::RTTIntrospectionBase* introBase
#endif
                        );

    bool isInited(){ return _inited;}
    void sense();
    std::string getFrame(){ return _force_torque_frame;}

private:
    /**
     * @brief _force_torque_frame is the frame where the force/torque is measured
     */
    std::string _force_torque_frame;

    /**
     * @brief _sensor is a pointer to the force/torque sensor related to the _force_torque_frame
     */
    gazebo::sensors::ForceTorqueSensorPtr _sensor;

    bool _inited;

    boost::shared_ptr<wrench> _wrench_measured;
    RTT::DataFlowInterface& _ports;

    /**
     * @brief pairFrameToSensor check if a sensor is associated to a given frame
     * @param joint_srdf is the joint name specified in the srdf under the group "force_torque_sensors"
     * @param sensor
     * @return true if the sensor is in the specified frame
     */
    bool pairFrameToSensor(const std::string& joint_srdf,
                           const gazebo::sensors::SensorPtr sensor,
                           gazebo::physics::ModelPtr gazebo_model,
                           boost::shared_ptr<urdf::ModelInterface const> urdf_model);

    void setFeedback();

    void fillMsg()
    {
        _wrench_measured->sensor_feedback.forces[0] = _sensor->Force().X();
        _wrench_measured->sensor_feedback.forces[1] = _sensor->Force().Y();
        _wrench_measured->sensor_feedback.forces[2] = _sensor->Force().Z();
        _wrench_measured->sensor_feedback.torques[0] = _sensor->Torque().X();
        _wrench_measured->sensor_feedback.torques[1] = _sensor->Torque().Y();
        _wrench_measured->sensor_feedback.torques[2] = _sensor->Torque().Z();
    }

#ifdef USE_INTROSPECTION
    cogimon::RTTIntrospectionBase* _introBase;
#endif
};

#endif
