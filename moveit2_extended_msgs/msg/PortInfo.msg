# One input or output port of a Behavior. This is what lets the browser editor offer a node's
# ports without hard-coding any of them: the server reads them straight out of the BehaviorTree.CPP
# factory manifest.
uint8 INPUT=0
uint8 OUTPUT=1
uint8 INOUT=2

string name
uint8 direction
# Demangled C++ type, for display. Not machine-checked: BehaviorTree.CPP resolves port types at
# read time, not at edit time.
string type_name
string default_value
string description
