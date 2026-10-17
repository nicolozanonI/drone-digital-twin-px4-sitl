# CustomVehicleOdometry.msg

# Sequence number custom
uint32 seq

# Campi da VehicleOdometry di PX4
uint64 timestamp
uint64 timestamp_sample

float32[3] position
float32[4] q

float32[3] velocity
float32[3] angular_velocity

float32[3] position_variance
float32[3] orientation_variance
float32[3] velocity_variance

uint8 pose_frame
uint8 velocity_frame