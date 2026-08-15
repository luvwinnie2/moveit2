# One Objective the server knows about.
string name
string description
string source_file
string[] required_parameters
string[] optional_parameters
string[] subtree_ids
string[] referenced_behaviors
# Non-empty means this Objective cannot currently be built: some node it references is not
# registered. Reported here so a UI can grey it out rather than letting the user run it and get an
# XML error.
string[] missing_behaviors
