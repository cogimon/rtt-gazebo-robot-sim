#include <imu.h>

imu_sensor::imu_sensor(const string &imu_frame,
                       gazebo::sensors::Sensor_V sensors, RTT::DataFlowInterface &ports):
        _imu_frame(imu_frame),
        _sensor(),
        _inited(false),
        _ports(ports)
{
    RTT::log(RTT::Info) << "Creating IMU Sensor "<<RTT::endlog();

    for(unsigned int i = 0; i < sensors.size(); ++i)
    {
        if(sensors[i]->Type().compare("imu") == 0)
        {
            _sensor = std::static_pointer_cast<gazebo::sensors::ImuSensor>(sensors[i]);

            std::string type = "";
#if GAZEBO_MAJOR_VERSION >= 8
            type = _sensor->Type();
#else
            type = _sensor->GetType();
#endif

            RTT::log(RTT::Info)<<"Sensor "<<_sensor->Name()<<" of type "<<type<<
                                 " specified in link "<<_imu_frame<<RTT::endlog();
        }

        if(_sensor != NULL)
            break;
    }

    if(_sensor == NULL)
        RTT::log(RTT::Error)<<_imu_frame<<" can not be associated to any IMU sensor"<<RTT::endlog();
    else{
        setFeedback();
        _inited = true;
    }

}

void imu_sensor::sense()
{
    if(_imu_measured)
    {
        fillMsg();
        if (_imu_measured->orocos_port.connected())
            _imu_measured->orocos_port.write(_imu_measured->sensor_feedback);
    }

}

void imu_sensor::setFeedback()
{
    _imu_measured.reset(new imu);
    _imu_measured->orocos_port.setName(_imu_frame+"_SensorFeedback");
    _imu_measured->orocos_port.doc("Orientation, angualar velocity and linear acceleration from imu sensor.");
    _ports.addPort(_imu_measured->orocos_port);

    _imu_measured->sensor_feedback = rstrt::robot::IMU();
    fillMsg();
    _imu_measured->orocos_port.setDataSample(_imu_measured->sensor_feedback);
}
