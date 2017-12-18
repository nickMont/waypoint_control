This package converts a full path trajectory (published as a PVATrajectory message) to PVA_Ref messages for use with the px4_control package.

Dependencies:
px4_control
app_pathplanner_interface

The two aforementioned packages must be built *before* building this package.

This node also communicates with the parameter changing service of px4_control.
UP on dpad selects high-damping hover parameters
DOWN on dpad selects low-damping aggressive flight parameters
X defaults to hover (this can be changed in the launch file). If the mode is misspelled or unrecognizable, hover is used.
B selects path-following parameters

Note that the quad will use onboard control if Local Position Mode is not selected in px4_control (LEFT on dpad by default)
