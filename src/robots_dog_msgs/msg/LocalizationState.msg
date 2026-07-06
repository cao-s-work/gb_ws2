std_msgs/Header header       # timestamp in the header is the time the measurement was taken
uint8 status                 # Localization status: 0=Init, 1=Map Loading, 2=Init Localization, 3=Continuous Loc, 4=Error 5=Dynamic Init Loc 6=Loc Lost
float64 rate                 # progress
string description           # message type
