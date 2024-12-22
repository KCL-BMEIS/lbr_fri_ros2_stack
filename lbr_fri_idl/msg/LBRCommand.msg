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
