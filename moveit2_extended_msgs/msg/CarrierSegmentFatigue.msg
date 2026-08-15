# How hard one link of the chain is worked over a whole run.
#
# A drag chain fails where it is bent REPEATEDLY, not where it is bent once, so the curvature swing
# and the reversal count matter more than any single-pose number.
#
# Caveat worth knowing before acting on `reversals`: it has a noise floor. The solver redistributes
# bend along the chain as the run's chord changes, and that crosses the direction dead-band even
# when the joints move one way only -- a 20-point one-way sweep already reports tens of reversals
# across the chain. Read it as a relative ranking between segments of one run, not as an absolute
# flex-cycle count.
uint32 segment
float64 min_curvature           # 1/m
float64 max_curvature           # 1/m
int32 reversals
