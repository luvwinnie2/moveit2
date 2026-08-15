# The carrier's state at one waypoint of a trajectory.
uint32 index
float64 time_from_start         # s; 0 when the caller supplied no timing
bool collision                  # the planning scene reports a collision at this state
CarrierDiagnostics diagnostics
