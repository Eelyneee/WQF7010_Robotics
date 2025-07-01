import cv2
import numpy as np
import mediapipe as mp
from keras.models import load_model

# Load Mini-Xception model
emotion_model = load_model("emotion_model.h5", compile=False)
 
emotion_labels = ['Angry', 'Disgust', 'Fear', 'Happy', 'Sad', 'Surprise', 'Neutral']
target_size = (64, 64)  # Input size for Mini-Xception

# Initialize MediaPipe Face Detection
mp_face = mp.solutions.face_detection
face_detection = mp_face.FaceDetection(model_selection=0, min_detection_confidence=0.5)

# Start Webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Press ESC to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Convert to RGB for MediaPipe
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_detection.process(rgb)

    if results.detections:
        for detection in results.detections:
            bbox = detection.location_data.relative_bounding_box
            ih, iw, _ = frame.shape
            x1 = int(bbox.xmin * iw)
            y1 = int(bbox.ymin * ih)
            w = int(bbox.width * iw)
            h = int(bbox.height * ih)
            x2, y2 = x1 + w, y1 + h

            # Ensure valid box
            x1, y1 = max(x1, 0), max(y1, 0)
            x2, y2 = min(x2, iw), min(y2, ih)

            face_img = frame[y1:y2, x1:x2]
            if face_img.size == 0:
                continue

            gray_face = cv2.cvtColor(face_img, cv2.COLOR_BGR2GRAY)
            resized_face = cv2.resize(gray_face, (48, 48))  # use 48x48 as required
            normalized_face = resized_face.astype("float32") / 255.0
            input_data = normalized_face.reshape(1, 48, 48, 1)  # shape: (1, 48, 48, 1)
            
            predictions = emotion_model.predict(input_data)
            # Only keep 'Happy' (index 3) and 'Neutral' (index 6)
            filtered = predictions[0][[3, 6]]
            filtered_labels = ['Smiling', 'Not Smiling']
            filtered_idx = np.argmax(filtered)
            label = filtered_labels[filtered_idx]

            # Draw bounding box and label
            color = (0, 255, 0) if label == "Smiling" else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{label}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    cv2.imshow("Smile Detection (Mini-Xception + MediaPipe)", frame)
    if cv2.waitKey(5) & 0xFF == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
