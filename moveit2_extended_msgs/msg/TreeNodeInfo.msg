# One node's place in the tree. Sent once per run, alongside the XML, so a UI can lay the tree out
# and then update it from BehaviorStatus alone.
uint8 ACTION=0
uint8 CONDITION=1
uint8 CONTROL=2
uint8 DECORATOR=3
uint8 SUBTREE=4
uint8 UNDEFINED=5

uint16 uid
# 0 for the root.
uint16 parent_uid
string instance_name
string registration_name
uint8 node_type
uint16[] children_uids
