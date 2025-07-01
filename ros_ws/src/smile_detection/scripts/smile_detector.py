#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Empty, String
import cv2
import numpy as np
import mediapipe as mp
from keras.models import load_model
import time
import subprocess

class SmileDetectorROS:
    def __init__(self):
        rospy.init_node('smile_detector_node')

        # Publishers
        self.smile_pub = rospy.Publisher('/smile_detected', Bool, queue_size=1)
        self.tts_pub = rospy.Publisher('/say_text', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/start_detection', Empty, self.start_detection_callback)
        rospy.Subscriber('/retry_signal', Empty, self.retry_callback)

        # Load Model and Initialize MediaPipe
        self.model = load_model("emotion_model.h5", compile=False)
        self.emotion_labels = ['Angry', 'Disgust', 'Fear', 'Happy', 'Sad', 'Surprise', 'Neutral']
        self.face_detector = mp.solutions.face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

        self.capture = None
        rospy.loginfo("SmileDetectorROS node ready.")

        self.launch_usb_cam()
        rospy.spin()

    def launch_usb_cam(self):
        try:
            subprocess.run(['roslaunch', 'usb_cam', 'usb_cam-test.launch'], check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to launch usb_cam: {e}")
        except FileNotFoundError:
            rospy.logerr("roslaunch not found. Make sure ROS is sourced properly.")

    def start_detection_callback(self, msg):
        self.detect_smile()

    def retry_callback(self, msg):
        self.detect_smile()

    def detect_smile(self):
        self.capture = cv2.VideoCapture(0)
        if not self.capture.isOpened():
            rospy.logerr("Camera not accessible.")
            return

        rospy.loginfo("Smile detection started.")
        self.tts_pub.publish("Smile please!")

        found_smile = False
        timeout = time.time() + 15  # 15 seconds timeout

        while time.time() < timeout and not found_smile:
            ret, frame = self.capture.read()
            if not ret:
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_detector.process(rgb)

            if results.detections:
                for detection in results.detections:
                    ih, iw, _ = frame.shape
                    bbox = detection.location_data.relative_bounding_box
                    x1 = int(bbox.xmin * iw)
                    y1 = int(bbox.ymin * ih)
                    x2 = x1 + int(bbox.width * iw)
                    y2 = y1 + int(bbox.height * ih)

                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(iw, x2), min(ih, y2)

                    face = frame[y1:y2, x1:x2]
                    if face.size == 0:
                        continue

                    gray_face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
                    resized_face = cv2.resize(gray_face, (48, 48))
                    normalized_face = resized_face.astype("float32") / 255.0
                    input_data = normalized_face.reshape(1, 48, 48, 1)

                    predictions = self.model.predict(input_data)
                    filtered = predictions[0][[3, 6]]  # Happy and Neutral
                    filtered_labels = ['Smiling', 'Not Smiling']
                    label = filtered_labels[np.argmax(filtered)]

                    if label == "Smiling":
                        found_smile = True
                        self.smile_pub.publish(Bool(True))
                        self.do_countdown()
                        break

        if not found_smile:
            self.smile_pub.publish(Bool(False))
            rospy.loginfo("No smile detected in time.")

        self.capture.release()
        cv2.destroyAllWindows()

    def do_countdown(self):
        for i in range(3, 0, -1):
            self.tts_pub.publish(str(i))
            time.sleep(1)
        self.tts_pub.publish("Say cheese!")

if __name__ == '__main__':
    try:
        SmileDetectorROS()
    except rospy.ROSInterruptException:
        pass
