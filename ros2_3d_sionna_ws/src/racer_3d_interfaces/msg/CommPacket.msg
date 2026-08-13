# Application payload types. Loss, delay and retries are applied by the
# communication emulator, never by DDS discovery or ROS graph connectivity.
uint8 TYPE_STATE=1
uint8 TYPE_MAP_CHUNK=2
uint8 TYPE_MAP_MANIFEST=3
uint8 TYPE_ALLOCATION_PROPOSAL=4
uint8 TYPE_ALLOCATION_ACK=5
uint8 TYPE_ALLOCATION_COMMIT=6
uint8 TYPE_ALLOCATION_COMMIT_ACK=7

builtin_interfaces/Time source_stamp
builtin_interfaces/Duration ttl
int32 sender_id
int32 receiver_id
int32 origin_id
uint64 sequence
uint8 payload_type
uint8 priority
bool reliable
uint32 payload_size
uint8[] payload
