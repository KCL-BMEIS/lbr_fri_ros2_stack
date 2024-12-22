# Wrench command for the Fast Robot Interface (FRI).
# Requires joint position and wrench overlay.
# Refer to KUKA::FRI::LBRCommand in friLBRCommand.h for documentation.
#
#  * joint_position [rad]
#  * wrench [N/Nm]

float64[7] joint_position
float64[6] wrench
