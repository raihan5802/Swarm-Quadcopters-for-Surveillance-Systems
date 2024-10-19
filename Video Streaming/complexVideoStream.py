import cv2
from ultralytics import YOLO
import supervision as sv
from collections import deque
from scipy.spatial import distance as dist
import numpy as np
from deepface import DeepFace
import threading
import queue
import time
import requests
from PIL import Image
from io import BytesIO

frame_width = 1280
frame_height = 720

class CentroidTracker:
    def __init__(self, maxDisappeared=50):
        self.nextObjectID = 0
        self.objects = {}
        self.disappeared = {}
        self.maxDisappeared = maxDisappeared

    def register(self, centroid):
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, rects):
        if len(rects) == 0:
            for objectID in list(self.disappeared.keys()):
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            return self.objects

        inputCentroids = np.zeros((len(rects), 2), dtype="int")
        for (i, (startX, startY, endX, endY)) in enumerate(rects):
            cX = int((startX + endX) / 2.0)
            cY = int((startY + endY) / 2.0)
            inputCentroids[i] = (cX, cY)

        if len(self.objects) == 0:
            for i in range(0, len(inputCentroids)):
                self.register(inputCentroids[i])
        else:
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())

            D = dist.cdist(np.array(objectCentroids), inputCentroids)

            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            usedRows = set()
            usedCols = set()

            for (row, col) in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue

                objectID = objectIDs[row]
                self.objects[objectID] = inputCentroids[col]
                self.disappeared[objectID] = 0

                usedRows.add(row)
                usedCols.add(col)

            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)

            if D.shape[0] >= D.shape[1]:
                for row in unusedRows:
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1

                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)
            else:
                for col in unusedCols:
                    self.register(inputCentroids[col])

        return self.objects

def extract_face(frame, bbox):
    x1, y1, x2, y2 = bbox
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    face = frame[y1:y2, x1:x2]
    return face

def detect_objects(frame_queue, result_queue, model):
    while True:
        frame = frame_queue.get()
        if frame is None:
            break
        result = model(frame, agnostic_nms=True)[0]
        result_queue.put(result)

def main():
    url = "http://100.81.229.108:5000/video_feed"

    model = YOLO("yolov8n.pt")  # Using a smaller, faster model
    box_annotator = sv.BoxAnnotator(
        thickness=2,
        text_thickness=2,
        text_scale=1
    )

    ct = CentroidTracker(maxDisappeared=50)
    previous_faces = deque(maxlen=2)

    frame_queue = queue.Queue(maxsize=5)
    result_queue = queue.Queue(maxsize=5)

    detection_thread = threading.Thread(target=detect_objects, args=(frame_queue, result_queue, model))
    detection_thread.start()

    stream = requests.get(url, stream=True)
    bytes_ = bytes()

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

                    if not frame_queue.full():
                        frame_queue.put(frame)

                    if not result_queue.empty():
                        result = result_queue.get()
                        detections = sv.Detections.from_yolov8(result)

                        person_class_id = 0  # class_id for "person"
                        detections = [d for d in detections if d[2] == person_class_id]

                        rects = []
                        for det in detections:
                            x1, y1, x2, y2 = det[0]
                            rects.append((int(x1), int(y1), int(x2), int(y2)))

                        objects = ct.update(rects)
                        labels = []

                        for (objectID, centroid) in objects.items():
                            text = f"ID {objectID}"
                            cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

                            for bbox in rects:
                                face = extract_face(frame, bbox)
                                if len(previous_faces) == 2:
                                    face1 = previous_faces[0]
                                    face2 = previous_faces[1]

                                    try:
                                        result = DeepFace.verify(face1, face2, model_name="Facenet", enforce_detection=False)
                                        if result["verified"]:
                                            text = f"ID {objectID} (Verified)"
                                            cv2.putText(frame, text, (centroid[0] - 10, centroid[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                                        else:
                                            text = f"ID {objectID} (Not Verified)"
                                            cv2.putText(frame, text, (centroid[0] - 10, centroid[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                                    except Exception as e:
                                        print(f"Face verification failed: {e}")

                                previous_faces.append(face)

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

    frame_queue.put(None)
    detection_thread.join()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

