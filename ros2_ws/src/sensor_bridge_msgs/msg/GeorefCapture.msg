string REASON_OK="ok"
string REASON_NO_POSE="no_pose"
string REASON_OUTSIDE_LINE="outside_line"
string REASON_OFF_LINE="off_line"
string REASON_OFF_HEIGHT="off_height"
string REASON_SATURATED="saturated"
SensorCapture raw
float32[] sample_s            # one entry per sample, m along the line; NaN without a pose
float32[] sample_d            # m, positive to the right of the heading
float32[] sample_h            # m above ground
bool[] sample_has_pose
float32 s                     # mean over samples with a pose; NaN if no sample has a pose
float32 d
float32 h
bool usable
string reason                 # one of REASON_*, first match in the order listed above after REASON_OK
