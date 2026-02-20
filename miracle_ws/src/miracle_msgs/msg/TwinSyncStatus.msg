# TwinSyncStatus - Digital twin synchronisation status per machine.
#
# Published by the sync_engine to report how well the digital twin
# tracks the physical machine state.

builtin_interfaces/Time timestamp

# Machine identifier
string machine_id

# Synchronisation quality score (0.0 = no sync, 1.0 = perfect)
float64 sync_quality

# Drift magnitude between physical and twin states (mm)
float64 drift_magnitude

# Per-axis drift values (one per axis, e.g. X, Y, Z, A, B)
float64[] axis_drift

# Latency between physical state and twin update (ms)
float64 sync_latency_ms

# True when drift exceeds the configured correction threshold
bool correction_active

# Number of corrections applied since last reset
uint32 corrections_applied
