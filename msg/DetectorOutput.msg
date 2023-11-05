# Output of the detectors
# The topic DetectorOutput is used to show the output from the detectors. 
# (offboard_detector: observer_detector)
# The topic DetectorOutput is exclusively used in the TII/SSRC Research Project performed by Purdue FDCHSL.

uint8 attack_detect                 # [-] attack detection signal (binary)
float32 detector_residual           # [-] 2-norm of the observer-based detector residual (Scalar)
float32 detector_threshold          # [-] threshold of the observer-based detector