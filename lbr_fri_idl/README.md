# lbr_fri_idl
Interface definition language (IDL) messages for the FRI.

## Command Interfaces
- `LBRJointPositionCommand.msg`:
    - Control modes:
        - `POSITION_CONTROL`
        - `JOINT_IMPEDANCE_CONTROL`
        - `CARTESIAN_IMPEDANCE_CONTROL`
    - Client command mode: `POSITION`
    - Contains:
        - Joint position
- `LBRTorqueCommand.msg`:
    - Control mode: `JOINT_IMPEDANCE_CONTROL`
    - Client command mode: `TORQUE`
    - Contains:
        - Joint position and torque overlay commands
- `LBRWrenchCommand.msg`: 
    - Control mode: `CARTESIAN_IMPEDANCE_CONTROL`
    - Client command mode: `WRENCH`
    - Contains:
        - Joint position and wrench overlay commands

- `LBRCommand.msg`:
    - Combined message to buffer the above

## State Interfaces
- `LBRState.msg`:
    - FRI exposed robot state
