# The set of K future trajectory candidates for one planning cycle, plus the
# selected best trajectory. Consumed by RViz, the benchmark harness, and the
# safety/selection layer. See docs/architecture.md sections 4.1 (steps 4-7) and
# 4.3 (Output Contract).

std_msgs/Header header

# All candidates generated this cycle (before and after safety filtering can be
# distinguished by `safe_flags`).
nav2_diffusion_msgs/TrajectoryCandidate[] candidates

# Per-candidate hard-safety result, parallel to `candidates`. True means the
# candidate passed the deterministic safety gate (collision, kinematic limits,
# stale-data checks). See docs/safety.md section 8.2.
bool[] safe_flags

# Per-candidate human-readable rejection reason, parallel to `candidates`.
# Empty string for accepted candidates. Used for diagnostics and benchmarking
# ("why did this candidate get dropped?").
string[] rejection_reasons

# Index into `candidates` of the selected best trajectory, or -1 if no safe
# candidate exists (the system then falls back per docs/safety.md section 8.4).
int32 best_index
