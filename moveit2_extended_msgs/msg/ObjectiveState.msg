# What the Objective server is doing right now. Published latched, so a panel that connects
# mid-run immediately knows the state instead of waiting for the next transition.
uint8 IDLE=0
uint8 RUNNING=1
uint8 SUCCEEDED=2
uint8 FAILED=3
uint8 CANCELED=4

builtin_interfaces/Time stamp
uint8 state
string tree_instance_id
string objective_name
# The deepest RUNNING leaf while state is RUNNING; empty otherwise.
string current_behavior
uint32 tick_count
