import cv2
import numpy as np
import json
from ultralytics import YOLO


model = YOLO("yolov8n.pt")

def process_jpeg(jpeg_bytes):
    """JPEG byte → detection JSON + drawn image"""
    img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
    results = model(img)[0]

    detections = []

    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy[0]
        cls = model.names[int(box.cls)]
        conf = float(box.conf)

        detections.append({
            "x": float(x1),
            "y": float(y1),
            "w": float(x2 - x1),
            "h": float(y2 - y1),
            "class": cls,
            "conf": conf
        })

        x1_i, y1_i = int(x1), int(y1)
        x2_i, y2_i = int(x2), int(y2)
        cv2.rectangle(img, (x1_i, y1_i), (x2_i, y2_i), (0, 255, 0), 2)
        label = f"{cls} {conf:.2f}"
        cv2.putText(img, label, (x1_i, y1_i - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    return img, json.dumps({"detections": detections})


def process_and_draw(jpeg_bytes):
    # JPEG → OpenCV image
    img = cv2.imdecode(
        np.frombuffer(jpeg_bytes, np.uint8),
        cv2.IMREAD_COLOR
    )

    results = model(img)[0]

    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cls = model.names[int(box.cls)]
        conf = float(box.conf)

	# Drawing box
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(
            img,
            f"{cls} {conf:.2f}",
            (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1
        )

    # Encoding jpeg
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ok:
        return None

    return buf.tobytes()


if __name__ == "__main__":
    with open("frame_00000.jpg", "rb") as f:
        jpeg_data = f.read()

    img, output_json = process_jpeg(jpeg_data)

    print(output_json)


    cv2.imshow("Detection Result", img)
    cv2.imwrite("output_detected.jpg", img)
    print("Kaydedildi → output_detected.jpg")

    cv2.waitKey(0)
    cv2.destroyAllWindows()
