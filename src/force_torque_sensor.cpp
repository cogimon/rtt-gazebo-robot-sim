#include <force_torque_sensor.h>
#include <gazebo-7/gazebo/sensors/Sensor.hh>
#include <rtt/Logger.hpp>

force_torque_sensor::force_torque_sensor(const std::string& joint_srdf,
                                         gazebo::physics::ModelPtr gazebo_model,
                                         boost::shared_ptr<urdf::ModelInterface const> urdf_model,
                                         gazebo::sensors::Sensor_V sensors,
                                         RTT::DataFlowInterface& ports):
    _force_torque_frame(""),
    _sensor(),
    _inited(false),
    _ports(ports)
{
    RTT::log(RTT::Info) << "Creating Force/Torque Sensor "<<RTT::endlog();

    for(unsigned int i = 0; i < sensors.size(); ++i)
    {
        if(sensors[i]->Type().compare("force_torque") == 0){
            if(pairFrameToSensor(joint_srdf, sensors[i], gazebo_model, urdf_model))
                _sensor = std::static_pointer_cast<gazebo::sensors::ForceTorqueSensor>(sensors[i]);
        }

        if(_sensor != NULL)
            break;
    }


    if(_sensor == NULL)
        RTT::log(RTT::Error)<<joint_srdf<<" can not be associated to any FT sensor"<<RTT::endlog();
    else{
        setFeedback();
        _inited = true;
    }
}

void force_torque_sensor::sense()
{
    if(_wrench_measured)
    {

        fillMsg();

        if (_wrench_measured->orocos_port.connected())
            _wrench_measured->orocos_port.write(_wrench_measured->sensor_feedback);
    }
}

void force_torque_sensor::setFeedback()
{
    _wrench_measured.reset(new wrench);
    _wrench_measured->orocos_port.setName(_force_torque_frame);
    _wrench_measured->orocos_port.doc("Wrench measured from force/torque sensor.");
    _ports.addPort(_wrench_measured->orocos_port);

    _wrench_measured->sensor_feedback = rstrt::dynamics::Wrench();
    fillMsg();
    _wrench_measured->orocos_port.setDataSample(_wrench_measured->sensor_feedback);
}

bool force_torque_sensor::pairFrameToSensor(const std::string& joint_srdf,
                                            const gazebo::sensors::SensorPtr sensor,
                                            gazebo::physics::ModelPtr gazebo_model,
                                            boost::shared_ptr<urdf::ModelInterface const> urdf_model)
{
    std::string sensor_joint_name = sensor->ParentName();

    ///Here we do 2 checks:
    /// 1. We check if joint_srdf and the sensor_joint_name are both parent joints of the same link
    std::string sensor_link_name = gazebo_model->GetJoint(sensor_joint_name)->GetChild()->GetName();
    boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_srdf);
    std::string srdf_link_name = urdf_joint->child_link_name;
    if(sensor_link_name.compare(srdf_link_name) == 0){
        RTT::log(RTT::Info)<<"Sensor "<<sensor->Name()<<" of type "<<sensor->GetType()<<
                             " specified in joint "<<sensor->ParentName()<<" is associated to link "<<
                             srdf_link_name<<RTT::endlog();
        _force_torque_frame = srdf_link_name;
        return true;
    }

    ///2. if not 1., we check if joint_srdf and the sensor_joint_name are one the parent joint and the other the child joint
    /// of the same link
    srdf_link_name = urdf_joint->parent_link_name;
    if(sensor_link_name.compare(srdf_link_name) == 0){
        RTT::log(RTT::Info)<<"Sensor "<<sensor->Name()<<" of type "<<sensor->GetType()<<
                             " specified in joint "<<sensor->ParentName()<<" is associated to link "<<
                             srdf_link_name<<RTT::endlog();
        _force_torque_frame = urdf_joint->child_link_name;
        return true;
    }
    return false;
}
