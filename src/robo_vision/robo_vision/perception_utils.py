from ultralytics import YOLO
import json
import numpy as np

# ================= LOAD MODEL =================
model = YOLO("yolov8n.pt")

SELECTED_CLASSES = [39, 41, 32]

# ================= LOAD CALIBRATION =================
# with open("calibration.json") as f:
#     calib = json.load(f)

# cam_matrix = np.array(calib["camera_matrix"])
# fx = cam_matrix[0, 0]
# fy = cam_matrix[1, 1]
# cx0 = cam_matrix[0, 2]

# IMG_WIDTH = calib["image_width"]

# ================= REAL OBJECT WIDTHS (meters) =================
# REAL_WIDTHS = {
#     "bottle":     0.07,
#     "cup":        0.08,
#     "cell phone": 0.07,
#     "mouse":      0.06,
#     "remote":     0.04,
#     "book":       0.15,
#     "ball": 0.22
# }

# ================= DETECTION =================
def detect(frame, imgsz=640, conf=0.25):
    results = model(
        frame,
        classes=SELECTED_CLASSES,
        imgsz=imgsz,
        conf=conf,
        verbose=False,
    )
    boxes = results[0].boxes
    detections = []

    for box in boxes:
        cls  = int(box.cls.item())
        conf = float(box.conf.item())
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        cx    = (x1 + x2) / 2
        cy    = (y1 + y2) / 2
        width = (x2 - x1)
        height = (y2 - y1)   


        detections.append({
            "class_id":    cls,
            "class":       model.names[cls],
            "confidence":  conf,
            "cx_pixel":    cx,
            "cy_pixel":    cy,
            "width_pixel": width,
            "height_pixel": height
        })

    return detections, results


# ================= TARGET =================
def get_target(detections, target_class=None):
    if len(detections) == 0:
        return None

    # Filter by requested class
    if target_class is not None:
        detections = [d for d in detections if d["class"] == target_class]

    if len(detections) == 0:
        return None

    return max(detections, key=lambda x: x["confidence"])


# ================= CORE MATH =================
def bb_area(x1, y1, x2, y2):
    """
    Calculate the area of a bounding box in pixels.

    Parameters:
        x1, y1: top-left corner of the bounding box
        x2, y2: bottom-right corner of the bounding box

    Returns:
        area in pixels^2
    """
    width = x2 - x1
    height = y2 - y1
    return width * height


def normalized_offset(cx_pixel, image_width):
    cx0 = image_width / 2
    
    # Map pixel position relative to center, divide by half width
    offset = (cx_pixel - cx0) / (cx0)
    return offset

# ================= SIGNAL =================
def compute_signal(target, frame_width):
    
    if target is None:
        return {"found": False}

    # Bounding box coordinates
    x1 = target["cx_pixel"] - target["width_pixel"]/2
    y1 = target["cy_pixel"] - target["height_pixel"]/2
    x2 = target["cx_pixel"] + target["width_pixel"]/2
    y2 = target["cy_pixel"] + target["height_pixel"]/2

    # Bounding box area
    area_pixel = bb_area(x1, y1, x2, y2)

    # Estimated distance using fitted power law based on class
    class_name = target["class"]
    if class_name == "bottle":
        distance = 49287.098 * (area_pixel ** (-0.566))
    elif class_name == "cup":
        distance = 31099.565 * (area_pixel ** (-0.550))
    elif class_name == "sports ball":
        distance = 20148.510 * (area_pixel ** (-0.541))
    else:
        # Fallback, though we only have these three
        distance = 0

    # Normalized horizontal offset [-1, 1]
    offset_x = normalized_offset(target["cx_pixel"], frame_width)

    return {
        "found": True,
        "class": target["class"],
        "area_pixel": area_pixel,
        "distance": distance,
        "offset_x": offset_x,
        "confidence": target["confidence"]
    }