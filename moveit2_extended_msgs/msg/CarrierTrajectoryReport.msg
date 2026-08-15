# Whether a whole trajectory is survivable by the cable carrier, and where it is worst.
std_msgs/Header header
string carrier_name
float64 bend_radius             # m, the hardware limit this was judged against
float64 bend_utilisation_warn   # the threshold used for "over-worked"

CarrierWaypointResult[] waypoints   # empty when only the summary was asked for
CarrierSegmentFatigue[] segments

int32 infeasible_count          # the carrier cannot take the required shape
int32 colliding_count
int32 over_worked_count         # above bend_utilisation_warn, but still possible
int32 cable_violation_count     # an inner cable below its own minimum bend radius

float64 worst_bend_utilisation
float64 worst_twist_utilisation
float64 worst_cable_strain
float64 worst_cable_bend_ratio
float64 required_cable_bend_ratio
string worst_cable
float64 tightest_radius         # m
bool has_tightest_radius        # false when nothing was feasible; 0 is a legitimate value for a
                                # perfectly straight run, so it cannot double as "never set"

uint32 hottest_segment
float64 hottest_segment_min_curvature
float64 hottest_segment_max_curvature
int32 hottest_segment_reversals
bool has_hottest_segment

int32 first_bad_index           # first waypoint that failed outright; -1 when clean

# Nothing infeasible, colliding or cable-violating. Being over-worked is deliberately NOT a
# failure: it is a warning, and the Objective decides what to do about it.
bool safe
string verdict                  # the one-line human sentence
