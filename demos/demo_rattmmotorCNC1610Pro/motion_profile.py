"""
Determine a trapezoidal motion profile to test the movement of a single axis of 
the CNC 1610 Pro.
"""
from pyberryplc.motion import TrapezoidalProfile

profile = TrapezoidalProfile(
    ds_tot=720.0,  # deg
    a_m=1000.0,    # deg/s2
    v_m=180.0,     # deg/s
    v_i=0.0,
    v_f=0.0
)

vel_plot = TrapezoidalProfile.plot_velocity_profiles(profile)
vel_plot.show()
