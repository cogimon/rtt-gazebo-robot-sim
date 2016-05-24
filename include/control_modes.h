#ifndef _CONTROL_MODES_H_
#define _CONTROL_MODES_H_

#include <string>
#include <rtt/Port.hpp>

namespace cogimon {

    class jointCtrlModes {
    public:
        enum ControlModes {
            JointPositionCtrl,
            JointTorqueCtrl,
            JointImpedanceCtrl
        };

        static constexpr const char* positionCtrlPort = "JointPositionCommand";
        static constexpr const char* impedanceCtrlPort = "JointImpedanceCommand";
        static constexpr const char* torqueCtrlPort = "JointTorqueCommand";
    };

    class jointFeedbackModes {
    public:
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
