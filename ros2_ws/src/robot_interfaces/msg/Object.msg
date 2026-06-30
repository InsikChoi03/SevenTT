# Tracked object in field frame (world model entry).
uint64 id                       # tracker id
string class_label              # "cube" | "icosahedron" | "apple" | ...
uint8 set_type                  # 0=unknown, 1=set1 (shape), 2=set2 (fruit), 3=storage_flag
float32 x                       # m, field frame
float32 y                       # m, field frame
float32 confidence              # 0.0 ~ 1.0
builtin_interfaces/Time last_seen
bool blacklisted                # passed (non-target or low confidence)
