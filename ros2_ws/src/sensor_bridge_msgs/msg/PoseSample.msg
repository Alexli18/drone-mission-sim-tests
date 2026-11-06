# A vehicle pose that entered a release decision of the georeferencing node.
uint64 t_us                   # PX4 timestamp_sample (simulated time)
float64 x_m                   # NED north from home, vehicle origin (offset not applied)
float64 y_m                   # NED east
float64 z_m                   # NED down
float32 heading_rad
float64 ground_z_m            # ground reference z0 (NED down) the release used: h = -(z - z0) - o_z
