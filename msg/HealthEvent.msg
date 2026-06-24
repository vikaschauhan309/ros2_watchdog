# Emitted on every topic health state transition (OK <-> STALE <-> DEAD).
uint8 STATUS_OK = 0
uint8 STATUS_STALE = 1
uint8 STATUS_DEAD = 2

string topic_name
uint8 status
string message
builtin_interfaces/Time stamp
