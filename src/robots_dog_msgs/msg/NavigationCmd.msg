builtin_interfaces/Time stamp
uint8 source
uint8 mc_mode_cmd
geometry_msgs/Twist vel_cmd
geometry_msgs/Pose pos_cmd

# source Enum
uint8 SOURCE_NAV=0
uint8 SOURCE_ARC=1

# mc_mode_cmd Enum
uint8 MC_MODE_STANDBY=0
uint8 MC_MODE_NAV_VEL_CTRL=1
uint8 MC_MODE_ARC_VEL_CTRL=2
uint8 MC_MODE_POS_CTRL=3
uint8 MC_MODE_SU_CTRL=4
uint8 MC_MODE_CHARGING=5
uint8 MC_MODE_PASSIVE=6