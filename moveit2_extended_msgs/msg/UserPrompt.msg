# A Behavior asking the operator something and blocking until it gets an answer.
#
# Published latched so a UI opened after the prompt appeared still sees it. An empty prompt_id
# means "the previous prompt is withdrawn" -- published when the Behavior is halted, so the UI
# drops the buttons instead of leaving a dead dialog on screen.
string prompt_id
string message
# One button per entry.
string[] choices
# True when a trajectory was published alongside this prompt for the operator to look at.
bool has_trajectory
# Seconds before the Behavior gives up and fails. 0 = wait forever.
float64 timeout
