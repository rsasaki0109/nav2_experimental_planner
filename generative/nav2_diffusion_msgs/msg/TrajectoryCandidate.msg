# A single time-indexed SE(2) trajectory candidate produced by a generative model.
# See docs/architecture.md section 4.4 (Trajectory Representation): the primary
# internal representation is a time-indexed SE(2) future pose sequence, NOT a raw
# velocity command. cmd_vel is derived later by the Command Extractor.

std_msgs/Header header

# Poses sampled along the candidate horizon, expressed in header.frame_id
# (typically base_link or odom per the model manifest frame_convention).
geometry_msgs/Pose[] poses

# Time offset of each pose relative to header.stamp. Parallel to `poses`
# (time_offsets[i] corresponds to poses[i]).
builtin_interfaces/Duration[] time_offsets

# Optional per-pose body-frame velocities, parallel to `poses`. May be empty
# when the model outputs poses only (velocities are then estimated downstream).
geometry_msgs/Twist[] velocities

# Generative model score / unnormalized likelihood for this candidate.
# Higher means more preferred by the learned prior. This is a soft prior signal
# only; it does NOT imply safety. Safety is decided by the deterministic gate.
float64 model_score
