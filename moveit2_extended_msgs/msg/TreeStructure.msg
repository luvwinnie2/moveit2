# The shape of a tree that is about to run, published latched when the tree is built.
#
# This is what replaces Groot for us. BehaviorTree.CPP v3's Tree exposes only a flat vector of
# nodes, so the parent/child graph here is reconstructed by the server before publishing.
builtin_interfaces/Time stamp
string tree_instance_id
string objective_name
# The source XML, so an editor can show and edit exactly what ran.
string xml
# Depth-first, parent before child.
TreeNodeInfo[] nodes
