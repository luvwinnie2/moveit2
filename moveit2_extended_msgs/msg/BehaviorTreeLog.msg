# Batched status changes since the previous message.
#
# Shaped like nav2_msgs/BehaviorTreeLog so existing habits carry over, but extended with the node
# uid and registration name -- nav2's version has neither, and a UI cannot address a node without
# the uid.
builtin_interfaces/Time stamp
string tree_instance_id
string objective_name
BehaviorStatus[] event_log
