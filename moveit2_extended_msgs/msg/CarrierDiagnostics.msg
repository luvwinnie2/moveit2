# How hard a cable carrier (dresspack) is being worked at one robot pose.
#
# The numbers a cable-simulation tool is expected to report -- bending, torsion and tension --
# expressed as utilisations rather than as stresses. A stress figure would be meaningless here: the
# carrier is an articulated shell that bends at its joints, so beam theory on it produces numbers
# far past the yield of any polymer it is made from. What actually governs service life is how
# close the chain is to its own bend limit, and how tightly the cables INSIDE it are being bent.
string carrier_name
bool feasible

float64 min_bend_radius         # m, the tightest radius actually reached
float64 bend_utilisation        # max_curvature * bend_radius; 1.0 = at the hardware stop
float64 twist_utilisation       # fraction of the per-link torsion stop used

# The cable industry sizes continuous-flex cable at about 10x its outer diameter, and qualified
# high-flex constructions at about 7.5x in compact runs. Going below the validated figure turns a
# ten-million-cycle life into a fraction of it, which is why this is tracked separately from the
# chain's own limit -- the cable is usually the binding constraint, not the carrier.
float64 cable_bend_ratio            # achieved radius / worst cable OD
float64 required_cable_bend_ratio   # what that cable's own spec demands; 0 = no cables declared
string worst_cable
float64 cable_strain                # (OD/2)/R, dimensionless
bool cables_within_limit

float64 axial_strain            # fraction of nominal length; a chain should never leave 0
float64 tension                 # N
float64 max_penetration         # m, deepest remaining overlap with a robot link
string penetrating_obstacle
