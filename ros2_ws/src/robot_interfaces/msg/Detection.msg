# Single detection from YOLO-World (image-frame bounding box + class label).
string label              # e.g. "apple", "icosahedron", "object"
float32 confidence        # 0.0 ~ 1.0
float32 x_center          # pixel
float32 y_center          # pixel
float32 width             # pixel
float32 height            # pixel
string source_frame       # camera_top | camera_body
