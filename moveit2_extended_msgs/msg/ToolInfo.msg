# One end-effector the robot can be fitted with.
string name
string description
string package_name
string urdf_file_path
# Where the tool's working point sits relative to the mount link. Cartesian Behaviors compose this
# away before solving, which is what lets the TCP change without touching the URDF or restarting
# move_group -- neither of which can change an SRDF group tip at run time anyway.
geometry_msgs/Pose tcp_offset
string[] allowed_collision_links
# The dresspack configuration that goes with this tool. A heavier or longer tool works the carrier
# harder, so the two belong together.
string carrier_config
float64 mass                    # kg, 0 when not declared
bool attached                   # true for the tool currently fitted
