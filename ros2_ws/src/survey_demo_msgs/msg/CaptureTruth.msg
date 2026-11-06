# Simulation only: where the sensor really was when it took each sample. The nodes
# under test never subscribe to /sim/* topics (the test checks that through rosapi).
uint64 capture_id
uint64[] sample_true_us       # true (PX4 simulated) time of each sample
uint64[] sample_stamp_us      # the stamp sent with it: true time + clock offset + jitter
float32[] sample_s            # PX4's pose at the true time, projected on the line; NaN without a pose
float32[] sample_d
