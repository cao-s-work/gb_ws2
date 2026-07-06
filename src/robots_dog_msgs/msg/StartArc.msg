std_msgs/Header header
uint8 cmd
uint8 secondary_cmd
uint8 tag_id
string map_path
geometry_msgs/Pose[] rth_goals
geometry_msgs/Vector3 speed
string extra_info

# CMD Enum
uint8 CMD_NONE=0
uint8 CMD_INITIAL=1
uint8 CMD_START=2
uint8 CMD_PAUSE=3
uint8 CMD_CONTINUE=4
uint8 CMD_STOP=5

# CMD Enum
uint8 SEC_CMD_NONE=0
uint8 SEC_CMD_AUTO=1
uint8 SEC_CMD_CALIBRATION=2
uint8 SEC_CMD_RTH=3
uint8 SEC_CMD_COARSE=4
uint8 SEC_CMD_CHARGED_EXIT=5