#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import mediapipe as mp
from keras.models import load_model
import time
import os
import subprocess

class SmileDetectorRealTime:
    def __init__(self):
        rospy.init_node('smile_detector_node')

        self.face_detector = mp.solutions.face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)
        self.bridge = CvBridge()
        self.detection_active = False 
        self.smile_classification_active = False
        self.face_detected_init_sent = False
        self.model_loaded = False
        self.timeout_duration = 60 
        self.start_time = None
        self.photo_dir = "/home/mustar/smily_bot_ws/tmp/smile_photos"
        if not os.path.exists(self.photo_dir):
            os.makedirs(self.photo_dir)
        self.last_annotated_frame = None

        self.smile_pub = rospy.Publisher('/smile_detected', Bool, queue_size=1)
        self.annotated_pub = rospy.Publisher('/annotated_image', Image, queue_size=1)
        self.camera_ready_pub = rospy.Publisher('/camera_ready', String, queue_size=1)
        self.image_view_ready_pub = rospy.Publisher('/image_view_ready', String, queue_size=1)
        self.face_detected_init_pub = rospy.Publisher('/face_detection_started', Empty, queue_size=1)

        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.Subscriber('/start_detection', String, self.start_detection_callback) # This handles all control signals

        # Load model
        model_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'emotion_model.h5')
        try:
            self.model = load_model(model_path, compile=False)
            self.model_loaded = True
            rospy.loginfo("Smile model loaded.")
        except Exception as e:
            rospy.logerr(f"Error loading model: {e}")

        rospy.loginfo("Real-time SmileDetector running.")
        rospy.spin()
    
    def start_detection_callback(self, msg):
        if msg.data == "init_camera":
            try:
                self.usb_cam_proc = subprocess.Popen(['rosrun', 'usb_cam', 'usb_cam_node','_video_device:=/dev/video2',  
                '_pixel_format:=yuyv', '_image_width:=640', '_image_height:=480'])
                rospy.loginfo("usb_cam_node launched.")
                rospy.sleep(2)
                self.camera_ready_pub.publish("camera ready")
            except Exception as e:
                rospy.logerr(f"Failed to launch usb_cam_node: {e}")

            try:
                self.image_view_proc = subprocess.Popen(
                    ['rosrun', 'image_view', 'image_view', 'image:=/annotated_image']
                )
                rospy.loginfo("Launched image_view for annotated_image.")
                rospy.sleep(1)
                self.image_view_ready_pub.publish("image_view ready")
            except Exception as e:
                rospy.logerr(f"Failed to launch image_view: {e}")
                
            self.detection_active = True
            self.smile_classification_active = False 
            self.face_detected_init_sent = False
            self.start_time = time.time()
            rospy.loginfo("Smile detection process initiated. Waiting for faces.")

        elif msg.data == "activate_smile_classification":
            rospy.loginfo("Received 'activate_smile_classification'. Starting smile recognition.")
            self.smile_classification_active = True

        elif msg.data == "stop":
            rospy.loginfo("Smile detection stopped.")
            self.detection_active = False
            self.smile_classification_active = False
            self.face_detected_init_sent = False

            rospy.sleep(5)
            if hasattr(self, 'image_view_proc') and self.image_view_proc:
                self.image_view_proc.terminate()
                rospy.loginfo("Terminated image_view.")
                self.image_view_proc = None
            if hasattr(self, 'usb_cam_proc') and self.usb_cam_proc:
                self.usb_cam_proc.terminate()
                rospy.loginfo("Terminated usb_cam_node.")
                self.usb_cam_proc = None
            


    def _process_frame(self, msg):
        """Converts ROS Image to CV2, processes with MediaPipe Face Detection."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return None, None

        output_frame = frame.copy()
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_detector.process(rgb)
        return output_frame, results.detections

    def _draw_initial_detection_boxes(self, output_frame, detections):
        """Draws yellow boxes and 'Detecting...' text during initial face detection phase."""
        for detection in detections:
            ih, iw, _ = output_frame.shape
            bbox = detection.location_data.relative_bounding_box
            x1 = int(bbox.xmin * iw)
            y1 = int(bbox.ymin * ih)
            x2 = x1 + int(bbox.width * iw)
            y2 = y1 + int(bbox.height * ih)

            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(iw, x2), min(ih, y2)
            
            if x2 - x1 > 0 and y2 - y1 > 0:
                cv2.rectangle(output_frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(output_frame, "Detecting...", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

    def _classify_smiles_and_annotate(self, output_frame, detections):
        """
        Performs smile classification for detected faces and annotates the frame.
        Returns True if all faces are smiling, False otherwise.
        """
        all_labels = []
        for detection in detections:
            ih, iw, _ = output_frame.shape
            bbox = detection.location_data.relative_bounding_box
            x1 = int(bbox.xmin * iw)
            y1 = int(bbox.ymin * ih)
            x2 = x1 + int(bbox.width * iw)
            y2 = y1 + int(bbox.height * ih)

            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(iw, x2), min(ih, y2)

            face = output_frame[y1:y2, x1:x2]
            if face.size == 0 or face.shape[0] == 0 or face.shape[1] == 0:
                continue

            gray_face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
            resized_face = cv2.resize(gray_face, (48, 48))
            normalized_face = resized_face.astype("float32") / 255.0
            input_data = normalized_face.reshape(1, 48, 48, 1)

            predictions = self.model.predict(input_data, verbose=0)
            happy_prob = predictions[0][3]
            neutral_prob = predictions[0][6]
            
            if happy_prob > neutral_prob and happy_prob > 0.5:
                 label = 'Smiling'
            else:
                 label = 'Not Smiling'

            all_labels.append(label)

            color = (0, 255, 0) if label == "Smiling" else (0, 0, 255)
            cv2.rectangle(output_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(output_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        return all_labels and all(label == "Smiling" for label in all_labels)

    def _publish_annotated_image(self, output_frame):
        """Publishes the annotated image frame."""
        self.last_annotated_frame = output_frame.copy()  # Store the latest frame
        try:
            self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(output_frame, encoding='bgr8'))
        except CvBridgeError as e:
            rospy.logerr(f"Error publishing annotated image: {e}")

    def image_callback(self, msg):
        if not self.detection_active or self.start_time is None:
            return   

        output_frame, detections = self._process_frame(msg)
        if output_frame is None: 
            return

        detected_smile = False

        if detections: # If faces are detected
            if not self.smile_classification_active:
                self._draw_initial_detection_boxes(output_frame, detections)
                if not self.face_detected_init_sent:
                    rospy.loginfo("Face detected. Signalling InteractionManager to start countdown.")
                    self.face_detected_init_pub.publish(Empty())
                    self.face_detected_init_sent = True
            else:
                detected_smile = self._classify_smiles_and_annotate(output_frame, detections)
        
        # Calculate how many seconds remain until timeout
        elapsed_time = time.time() - self.start_time
        remaining_time = max(0, 30 - int(elapsed_time))
        cv2.putText(output_frame, 
                    f"Time remaining: {remaining_time}s", 
                    (10, 40),  # Position (x, y)
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    1,  # Font scale
                    (0, 255, 255),  # Yellow color
                    2)  # Thickness

        self._publish_annotated_image(output_frame)
        if self.smile_classification_active:
            if detected_smile:
                self.handle_smile_event(True)  # All smiling, stop session
            else:
                # Check for timeout
                if time.time() - self.start_time > 30: 
                    rospy.loginfo("Smile classification timed out. Asking user to retry.")
                    self.detection_active = False
                    self.smile_classification_active = False
                    self.smile_pub.publish(Bool(False))
        else:
            pass

    def handle_smile_event(self, detected_smile):
        rospy.loginfo("Smile detected! Stopping detection and preparing for photo process.")
        self.smile_pub.publish(Bool(detected_smile))
        self.detection_active = False

        if detected_smile and self.last_annotated_frame is not None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"photo_{timestamp}_annotation.jpg"
            file_path = os.path.join(self.photo_dir, filename)
            
            try:
                cv2.imwrite(file_path, self.last_annotated_frame)
                rospy.loginfo(f"Saved smile photo to {file_path}")
            except Exception as e:
                rospy.logerr(f"Error saving photo: {e}")

        rospy.loginfo("Smile detection session concluded.")


if __name__ == '__main__':
    try:
        SmileDetectorRealTime()
    except rospy.ROSInterruptException:
        pass