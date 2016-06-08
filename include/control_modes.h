#ifndef _CONTROL_MODES_H_
#define _CONTROL_MODES_H_

#include <string>
#include <rtt/Port.hpp>

namespace cogimon {

    struct ControlModes{
            static constexpr const char* JointPositionCtrl = "JointPositionCtrl";
            static constexpr const char* JointTorqueCtrl = "JointTorqueCtrl";
            static constexpr const char* JointImpedanceCtrl = "JointImpedanceCtrl";
    };

    struct FeedbackModes {
        static constexpr const char* velocityFeedback = "JointVelocity";
        static constexpr const char* torqueFeedback = "JointTorque";
        static constexpr const char* positionFeedback = "JointPosition";
    };


    template <class T> class jointCtrl {
    public:
        jointCtrl(){

        }

        ~jointCtrl(){

        }

        RTT::InputPort<T> orocos_port;
        RTT::FlowStatus joint_cmd_fs;
        T joint_cmd;
    };


    template <class T> class jointFeedback {
    public:
        jointFeedback() {
        }

        ~jointFeedback() {

        }

        T joint_feedback;
        RTT::OutputPort<T> orocos_port;

    };
}
#endif
