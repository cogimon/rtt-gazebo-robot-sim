#ifndef _CONTROL_MODES_H_
#define _CONTROL_MODES_H_
#include <string>

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

    class jointFeedback {
    public:
        static constexpr const char* velocityFeedbackPort = "JointVelocity";
        static constexpr const char* torqueFeedbackPort = "JointTorque";
        static constexpr const char* positionFeedbackPort = "JointPosition";

    };
}
#endif
