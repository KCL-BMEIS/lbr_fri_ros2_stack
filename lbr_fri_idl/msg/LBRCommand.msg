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

string[2] analog_io_names
float64[2] analog_io_values 
string[2] boolean_io_names   
bool[2] boolean_io_values  
string[2] digital_io_names
uint64[2] digital_io_values 