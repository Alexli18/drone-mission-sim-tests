uint64 now_us                 # the node's newest pose time
bool pose_received
uint64 last_pose_stamp_us
uint32 captures_seen          # raw captures received
bool has_capture
uint64 last_capture_id        # highest raw capture id received
uint32 released               # captures released (equals lines in decisions.jsonl)
uint32 usable
string[] reasons              # every GeorefCapture REASON_* value, in the order of the constants
uint32[] counts               # parallel to reasons
uint32 poses_used             # messages published on /survey/pose_used
float32 coverage              # sensor_bridge.coverage over released captures
float32 max_gap_m
bool queue_empty              # no held captures
