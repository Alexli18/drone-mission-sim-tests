# One capture of the survey sensor, without pose: a few readings taken a few
# milliseconds apart, each with its own timestamp.
std_msgs/Header header        # stamp: sensor clock of the first sample; frame_id: sensor_id
string sensor_id
uint64 capture_id             # 0 for the first capture after start-up, then +1
uint64[] sample_stamps_us     # one entry per sample: sensor clock
float32[] readings            # one entry per sample, 0 to 1
bool saturated                # true if any reading is at full scale
