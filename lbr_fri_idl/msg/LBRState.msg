# Message that holds states that are exposed through the
# Fast Robot Interface (FRI).
# Following states are exposed (refer to KUKA::FRI::LBRState in friLBRState.h for documentation):
#
#  * sample_time [s]
#  * session_state [#]
#  * connection_quality [#]
#  * safety_state [#]
#  * operation_mode [#]
#  * drive_state [#]: 
#  * client_command_mode [#]
#  * overlay_type [#]
#  * control_mode [#]
#
#  * time_stamp_sec [s]
#  * time_stamp_nano_sec [ns]
#
#  * measured_joint_position [rad]
#  * commanded_joint_position [rad]
#  * measured_torque [Nm]
#  * commanded_torque [Nm]
#  * external_torque [Nm]
#  * ipo_joint_position [rad]
#  * tracking_performance [a.u.]

float64 sample_time
int8 session_state
int8 connection_quality
int8 safety_state
int8 operation_mode
int8 drive_state
int8 client_command_mode
int8 overlay_type
int8 control_mode

uint32 time_stamp_sec
uint32 time_stamp_nano_sec

float64[7] measured_joint_position
float64[7] commanded_joint_position
float64[7] measured_torque
float64[7] commanded_torque
float64[7] external_torque
float64[7] ipo_joint_position
float64 tracking_performance
