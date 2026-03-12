builtin_interfaces/Time timestamp
string machine_id
string feedback_type          # RECOMMENDATION_ACCEPTED, RECOMMENDATION_REJECTED, EXPLANATION_RATED, ACTION_CONFIRMED, ACTION_FAILED
string reference_id           # ID of the recommendation/explanation being rated
string anomaly_type           # what anomaly this relates to
float32 rating                # 1-5 scale for explanation quality, or 1.0/0.0 for accept/reject
string operator_comment       # optional text
string action_taken           # what the operator actually did
