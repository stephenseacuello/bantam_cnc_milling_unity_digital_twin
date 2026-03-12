builtin_interfaces/Time timestamp
string alarm_id
string machine_id
string alarm_type
int32 escalation_level
float32 severity
string escalation_action      # PRIORITY_BOOST, FORCED_ACK_REQUIRED, SUPERVISOR_NOTIFY, EMERGENCY_STOP
string reason                 # Why escalated
float32 time_unacknowledged_sec
