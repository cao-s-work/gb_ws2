std_msgs/Header header       # timestamp in the header is the time the measurement was taken
uint8 status                 # perception status: 0=Init, 1=Loading model, 2=sensor dump, 3=runing
float64 rate                 # hz
string description           # message type
