# Runtime safety state published by the safety/selection layer. Maps directly to
# the Safety State Machine in docs/safety.md section 8.3. Consumed by the BT /
# supervisor to drive fallback and stop decisions.

std_msgs/Header header

# safe candidates available, latency nominal -> cmd_vel from generative best traj
uint8 NOMINAL=0
# confidence low / dynamic risk high -> speed limited
uint8 CAUTIOUS=1
# model delayed / partial sensor loss -> previous safe trajectory or fallback
uint8 DEGRADED=2
# model disabled / no safe candidate -> MPPI / RPP / configured controller
uint8 FALLBACK=3
# imminent risk -> controlled stop
uint8 BRAKE=4
# collision monitor / hardware stop -> zero or hardware stop
uint8 EMERGENCY_STOP=5
# delegated to Nav2 recovery behavior (BT managed)
uint8 RECOVERY=6

uint8 state

# Human-readable explanation of the current state (e.g. "inference deadline miss",
# "costmap stale", "all candidates collide"). See docs/architecture.md section 7.4.
string detail
