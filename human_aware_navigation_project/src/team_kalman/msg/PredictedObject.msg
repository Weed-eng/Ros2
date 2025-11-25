# ===============================
#  Predicted moving object state
#  (all in base_link frame)
# ===============================

# Current estimated state
float32 x
float32 y
float32 vx
float32 vy

# Prediction at time horizon t_horizon
float32 x_pred
float32 y_pred
float32 t_horizon   # [s]

# Safety radius suggested around predicted position
float32 safety_radius

# Confidence (0.0 - 1.0), propagated from LiDAR + filter
float32 confidence
