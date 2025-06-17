# Sample code for reference

#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, Bool, String
import cv2
import time

class SmileDetector:
    def __init__(self):
        rospy.init_node('smile_detector')

        # Subscribers
        rospy.Subscriber('/start_detection', Empty, self.start_detection_callback)
        rospy.Subscriber('/retry_signal', Empty, self.retry_callback)

        # Publisher
        self.smile_pub = rospy.Publisher('/smile_detected', Bool, queue_size=1)
        self.tts_pub = rospy.Publisher('/say_text', String, queue_size=1)

        # Load Haar cascades for face and smile detection
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.smile_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_smile.xml')

        self.retry_limit = 3
        rospy.loginfo("SmileDetector node is ready.")
        rospy.spin()

    def start_detection_callback(self, msg):
        self.detect_smile()

    def retry_callback(self, msg):
        self.detect_smile()

    def detect_smile(self):
        cap = cv2.VideoCapture(0)
        retry_count = 0
        max_time = 15  # seconds
        found_smile = False

        while retry_count < self.retry_limit and not found_smile:
            rospy.loginfo(f"Smile detection attempt {retry_count + 1}")
            self.tts_pub.publish("Smile please!")

            start_time = time.time()

            while time.time() - start_time < max_time:
                ret, frame = cap.read()
                if not ret:
                    continue

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

                for (x, y, w, h) in faces:
                    roi_gray = gray[y:y+h, x:x+w]
                    smiles = self.smile_cascade.detectMultiScale(roi_gray, 1.8, 20)

                    if len(smiles) > 0:
                        self.do_countdown()
                        found_smile = True
                        self.smile_pub.publish(Bool(True))
                        rospy.loginfo("Smile detected!")
                        break

                if found_smile:
                    break

            retry_count += 1

        if not found_smile:
            self.smile_pub.publish(Bool(False))
            rospy.loginfo("Smile not detected after retries.")

        cap.release()
        cv2.destroyAllWindows()

    def do_countdown(self):
        for i in range(3, 0, -1):
            self.tts_pub.publish(str(i))
            time.sleep(1)

if __name__ == '__main__':
    try:
        SmileDetector()
    except rospy.ROSInterruptException:
        pass
