# ===============================
#  Custom LiDAR Moving Object Message
# ===============================

# Unique ID for this tracked object
int32 id

# Object position in robot frame (base_link)
float32 x
float32 y

# Polar position
float32 r
float32 theta

# Estimated velocity
float32 vx
float32 vy
float32 speed
float32 direction

# Object geometry
float32 radius
int32 num_points

# Tracking metadata
int32 age

# Confidence score (0.0 - 1.0)
float32 confidence
