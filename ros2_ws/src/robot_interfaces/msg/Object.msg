# Tracked object in field frame (world model entry).
uint64 id                       # tracker id
string class_label              # "cube" | "icosahedron" | "apple" | ...
uint8 set_type                  # 0=unknown, 1=set1 (shape), 2=set2 (fruit), 3=storage_flag
float32 x                       # m, field frame
float32 y                       # m, field frame
float32 confidence              # 0.0 ~ 1.0
builtin_interfaces/Time last_seen
bool blacklisted                # passed (non-target or low confidence)
uint32 n_obs                    # total detections fused into this track (evidence count)
string source                   # "wide" | "body" | "wide+body" (which cameras have seen it)
string fruit_label              # SigLIP best fruit for a Set2 box ("" until read)
float32 fruit_confidence        # confidence/margin of the selected body or wide fruit label
string fruit_label_source       # "body" | "wide" | "" (wide remains routing-only evidence)
bool locked                     # frozen as a pose-correction landmark (stable static object)
