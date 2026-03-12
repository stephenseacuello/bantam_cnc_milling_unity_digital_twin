# Inferred control action from cognitive reasoning engine
builtin_interfaces/Time timestamp
string machine_id
string inference_rule        # e.g. "overload_requires_feedrate_reduction"
string action_type           # REDUCE_FEED, REDUCE_SPEED, TOOL_CHANGE, COOLANT_INCREASE, PAUSE
string subject               # e.g. "cnc1"
string predicate             # e.g. "requires"
string object_value          # e.g. "FeedRateReduction"
float32 confidence           # 0.0-1.0
string reasoning             # Human-readable explanation
