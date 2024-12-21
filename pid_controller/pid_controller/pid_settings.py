import numpy as np
# PID controller parameters

# ROLL CONTROLLER 
# ---------------------
ROLL_KP =  7.25
ROLL_KD = 12.85
ROLL_KI = 5.2
# ----------------------


# PITCH CONTROLLER
# --------------------
PITCH_POS_KP = 88.0
PITCH_NEG_KP = 127.2
PITCH_KP_SIGMOID_K = 1.25

PITCH_POS_KD = 14.15
PITCH_NEG_KD = 20.5
PITCH_KD_SIGMOID_K = 1.25

PITCH_KI = 1.5
# ---------------------


# YAW CONTROLLER
# ---------------------
YAW_KP = 40.0
YAW_KD =  10.0
YAW_KI = 0.0
# ---------------------

# VERTICAL CONTROLLER 
# -------------------
Z_KP = 0.0
Z_KD = 0.0
Z_KI = 0.0
HOVER_THROTTLE = 100

# Assume x, y K values to be 0 for now 
KP_VECTOR = np.array([0.0, 0.0, Z_KP, ROLL_KP, 0.0, YAW_KP])
KD_VECTOR = np.array([0.0, 0.0, Z_KD, ROLL_KD, 0.0, YAW_KD])
KI_VECTOR = np.array([0.0, 0.0, Z_KI, ROLL_KI, PITCH_KI, YAW_KI])
