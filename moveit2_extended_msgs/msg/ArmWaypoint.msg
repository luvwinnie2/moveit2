# A named arm target.
#
# A waypoint may carry joint values, a Cartesian pose, or both. Joints win when a Behavior is not
# told otherwise: on an arm whose IK returns one branch only, replaying a pose is not guaranteed to
# reproduce the elbow the operator taught.
string name
string group                    # planning group it was taught for
string description
string[] tags

bool has_joint_state
sensor_msgs/JointState joint_state

bool has_pose
# The pose is stored WITH the frame and the link it describes, and neither is optional. On a mobile
# base, a pose taught in robot_base is a robot-RELATIVE target: replaying it after the base has
# driven somewhere else would send the arm to a different place in the world entirely.
geometry_msgs/PoseStamped pose
string pose_link

string tool                     # which end-effector was fitted when this was taught
bool has_carrier
CarrierDiagnostics carrier      # what the dresspack was doing here, recorded at teach time

builtin_interfaces/Time created
builtin_interfaces/Time modified
