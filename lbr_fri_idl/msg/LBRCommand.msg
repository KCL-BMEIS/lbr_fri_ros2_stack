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

string analog_name
float64[2] analog_io
string boolean_name
bool[2] boolean_io
string digital_name
uint64[2] digital_io