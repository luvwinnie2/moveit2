# One node of a running Behavior Tree changing state.
uint8 IDLE=0
uint8 RUNNING=1
uint8 SUCCESS=2
uint8 FAILURE=3
# BehaviorTree.CPP v3 never emits SKIPPED. It is reserved here so this message does not have to
# change when the implementation moves to v4, which does.
uint8 SKIPPED=4

builtin_interfaces/Time stamp
# Stable for the lifetime of one tree instance; the key a UI uses to address a node.
uint16 uid
# The XML name= attribute, or the tag when none was given.
string instance_name
# The Behavior type, i.e. what the loader registered it as.
string registration_name
uint8 previous_status
uint8 current_status
