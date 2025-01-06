# Message that holds commands that are exposed through the
# Fast Robot Interface (FRI).
# Following commands are exposed (refer to KUKA::FRI::LBRCommand in friLBRCommand.h for documentation):
#
#  * joint_position [rad]
#  * torque [Nm]
#  * wrench [N/Nm]

float64[7] joint_position
float64[7] torque
float64[6] wrench

string[2] analog_io
float64[2] analog_value
string[8] digital_io
uint64[8] digital_value
#  * string boolean_io
#  * unit64[8] boolean_value