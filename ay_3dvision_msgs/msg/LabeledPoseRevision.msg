# Optimized pose
LabeledPose lpose

# Revision status to check if the pose is updated, which is checked by the given thresholds.
#   0: Not revised at all.
#   1: More than one (but not all) optimization stages are succeeded.
#   2: All optimization stages are succeeded.
int32 revision_status

# Errors of the revision.
# See the source code of the node to know the definition.
float64[] errors

# Optimized parameters.
# Used when we are optimizing parameters on constrained axes.
float64[] params
