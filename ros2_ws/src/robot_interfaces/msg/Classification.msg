# Fine-grained classification / pick-gate result for a single body-cam detection.
# Produced by siglip_gate_node (fruit/cube discriminator) and shape_heuristic_node
# (shape edge-count). Consumed by mission_fsm_node in the CLASSIFY state to make the
# pick-vs-pass decision (rulebook §6/§7: mispick on Set2 = -40, so the gate is conservative).
std_msgs/Header header
string label              # best class label, "" if undecided
uint8 set_type            # 1=set1 (shape), 2=set2 (fruit), 0=unknown
float32 confidence        # 0.0 ~ 1.0
bool is_target            # passes the gate for today's announced target
bool image_face_visible   # Set2 discriminator: a fruit-picture face was detected on a cube
string source             # "siglip" | "shape_heuristic"
