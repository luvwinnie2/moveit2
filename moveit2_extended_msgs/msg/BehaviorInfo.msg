# One Behavior the server can build. Together these form the editor's node palette.
string registration_name
# TreeNodeInfo constants.
uint8 node_type
string description
PortInfo[] ports
# Which package's loader registered it. Makes an ID collision between two packages diagnosable
# from the command line instead of by reading server logs.
string loader_package
string loader_class
