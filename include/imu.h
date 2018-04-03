#ifndef _FORCE_IMU_SENSOR_H_
#define _FORCE_IMU_SENSOR_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <XBotCoreModel.h>
#include <rtt/Port.hpp>
#include <rst-rt/robot/IMU.hpp>
#include <gazebo/sensors/ImuSensor.hh>
#include <force_torque_sensor.h>

typedef sensorFeedback<rstrt::robot::IMU> imu;

/**
 * @brief The imu_sensor class considers the imu sensors specified in the urdf
 * file and broadcasts the data in the OROCOS framework. Normally, the imu sensor is
 * specified wrt a link of the model
 */
class imu_sensor{
public:
    imu_sensor(const std::string& imu_frame,
               gazebo::sensors::Sensor_V sensors,
               RTT::DataFlowInterface& ports);

    bool isInited(){ return _inited;}
    void sense();
    std::string getFrame(){ return _imu_frame;}

private:

    /**
     * @brief _force_torque_frame is the frame where the force/torque is measured
     */
    std::string _imu_frame;

    /**
     * @brief _sensor is a pointer to the force/torque sensor related to the _force_torque_frame
     */
    gazebo::sensors::ImuSensorPtr _sensor;

    bool _inited;

    boost::shared_ptr<imu> _imu_measured;
    RTT::DataFlowInterface& _ports;


    void setFeedback();

    void fillMsg()
    {
        //Angular Velocity
        _imu_measured->sensor_feedback.angularVelocity[0] = _sensor->AngularVelocity().X();
        _imu_measured->sensor_feedback.angularVelocity[1] = _sensor->AngularVelocity().Y();
        _imu_measured->sensor_feedback.angularVelocity[2] = _sensor->AngularVelocity().Z();

        //Linear Acc
        _imu_measured->sensor_feedback.linearAcceleration[0] = _sensor->LinearAcceleration().X();
        _imu_measured->sensor_feedback.linearAcceleration[1] = _sensor->LinearAcceleration().Y();
        _imu_measured->sensor_feedback.linearAcceleration[2] = _sensor->LinearAcceleration().Z();

        //Rotation (here I suppose the order is [qw,qx,qy,qz] from rst-rt/robot/IMU.hpp header
        _imu_measured->sensor_feedback.rotation[0] = _sensor->Orientation().W();
        _imu_measured->sensor_feedback.rotation[1] = _sensor->Orientation().X();
        _imu_measured->sensor_feedback.rotation[2] = _sensor->Orientation().Y();
        _imu_measured->sensor_feedback.rotation[3] = _sensor->Orientation().Z();
    }


};



#endif
