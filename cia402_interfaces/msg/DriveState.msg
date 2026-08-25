# EN 61800-7-201 profile type 1 states

uint8 STATE_NOT_READY_TO_SWITCH_ON=0
uint8 STATE_SWITCH_ON_DISABLED=1
uint8 STATE_READY_TO_SWITCH_ON=2
uint8 STATE_SWITCHED_ON=3
uint8 STATE_OPERATION_ENABLED=4
uint8 STATE_QUICK_STOP_ACTIVE=5
uint8 STATE_FAULT_REACTION_ACTIVE=6
uint8 STATE_FAULT=7
uint8 STATE_UNKNOWN=255

# Time at which the state was sampled
builtin_interfaces/Time stamp

# ros2_control joint name
string joint_name

# Raw drive feedback
uint16 status_word
int8 mode_of_operation_display
bool feedback_valid

# Decoded state
uint8 state

# Last values requested by the controller
uint16 commanded_control_word
int8 commanded_mode_of_operation

# Decoded Status Word flags
bool ready_to_switch_on
bool switched_on
bool operation_enabled
bool fault
bool voltage_enabled
# True only when the complete Statusword pattern decodes to Quick stop active.
# Bit 5 alone uses inverse logic but is also clear in several other states.
bool quick_stop_active
bool switch_on_disabled
bool warning
bool remote
bool target_reached
bool internal_limit_active
