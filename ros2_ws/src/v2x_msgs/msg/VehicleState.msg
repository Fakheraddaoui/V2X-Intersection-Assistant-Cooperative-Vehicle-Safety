uint32 vehicle_id
builtin_interfaces/Time stamp    # EtherCAT-DC synchronized
float32 px                       # m, intersection frame
float32 py
float32 vx                       # m/s
float32 vy
float32 length                   # vehicle bounding box, m
float32 width
uint8 INTENT_STRAIGHT=0
uint8 INTENT_LEFT=1
uint8 INTENT_RIGHT=2
uint8 INTENT_STOP=3
uint8 intent
