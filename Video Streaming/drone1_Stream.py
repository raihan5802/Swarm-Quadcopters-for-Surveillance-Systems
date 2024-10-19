import cv2
from ultralytics import YOLO
import supervision as sv
import requests
from PIL import Image
from io import BytesIO
import numpy as np

frame_width = 1280
frame_height = 720

# URL of the Raspberry Pi's video feed
url = "http://100.81.229.108:5000/video_feed"

def main():

    # Start the video stream from the URL
    stream = requests.get(url, stream=True)
    bytes_ = bytes()

    model = YOLO("yolov8m.pt")

    box_annotator = sv.BoxAnnotator(
        thickness=2,
        text_thickness=2,
        text_scale=1
    )

    while True:
        try:
            for chunk in stream.iter_content(chunk_size=1024):
                bytes_ += chunk
                a = bytes_.find(b'\xff\xd8')
                b = bytes_.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = bytes_[a:b+2]
                    bytes_ = bytes_[b+2:]
                    img = Image.open(BytesIO(jpg))
                    frame = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

                    result = model(frame, agnostic_nms=True)[0]
                    detections = sv.Detections.from_yolov8(result)

                    # Filter detections to include only the person class (class ID 0)
                    detections = [d for d in detections if d[2] == 0]

                    labels = [
                        f"{model.model.names[class_id]} {confidence:0.2f}"
                        for _, confidence, class_id, _
                        in detections
                    ]

                    frame = box_annotator.annotate(
                        scene=frame, 
                        detections=detections, 
                        labels=labels
                    )

                    cv2.imshow("yolov8", frame)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        except Exception as e:
            print(f"Error: {e}")
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

