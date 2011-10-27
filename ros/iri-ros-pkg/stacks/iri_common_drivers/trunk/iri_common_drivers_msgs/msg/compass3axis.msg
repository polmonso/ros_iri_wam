# timestamp, frame id
Header header

# Heading, Pitch, Roll
# [rad/s]
float64[3] angles

# Heading, Pitch, Roll
float64[9] covariances

# magnetic for heading, out of range for pitch & roll
bool[3] alarms
