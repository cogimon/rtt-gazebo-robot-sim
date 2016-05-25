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

    struct FeedbackModesPorts {
        static constexpr const char* velocityFeedbackPort = "JointVelocity";
        static constexpr const char* torqueFeedbackPort = "JointTorque";
        static constexpr const char* positionFeedbackPort = "JointPosition";
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
