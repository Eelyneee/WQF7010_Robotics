#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool # To subscribe to smile_detected
from sensor_msgs.msg import Image # To publish captured image
import cv2
from cv_bridge import CvBridge # To convert OpenCV image to ROS Image message
import os
import pygame.mixer
import time # For camera warm-up and delays

# --- Audio File Path ---
# Make sure 'beep.mp3' or 'beep.wav' is in the same directory as this script,
# or adjust path accordingly (e.g., in a 'audio' subfolder)
AUDIO_FILE_BEEP = os.path.join(os.path.dirname(__file__), "..", "audio", "beep.mp3") # Change to .wav if needed


class PlayBeepAndCaptureNode:
    def __init__(self):
        rospy.init_node('play_beep_and_capture', anonymous=True)

        # 1. Initialize Pygame Mixer
        try:
            pygame.mixer.init()
            pygame.mixer.music.set_volume(0.7) 
            rospy.loginfo("Pygame mixer initialized for PlayBeepAndCaptureNode.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Pygame mixer: {e}")
            rospy.signal_shutdown("Could not initialize sound system.")
            return

        # 2. Initialize Camera (for capturing photos)
        self.camera = cv2.VideoCapture(0) # This node opens its own camera
        if not self.camera.isOpened():
            rospy.logerr("PlayBeepAndCaptureNode: Failed to open camera. Image capture will not work.")
            # You might want to signal_shutdown here if camera is essential
        else:
            rospy.loginfo("PlayBeepAndCaptureNode: Camera opened successfully.")
            time.sleep(1) # Give camera a moment to warm up

        # 3. Initialize CvBridge
        self.bridge = CvBridge()

        # 4. Set up Publisher for captured images
        self.image_pub = rospy.Publisher('/save_photo', Image, queue_size=1) 

        # 5. Set up Subscriber for smile detection status
        rospy.Subscriber("/smile_detected", Bool, self.smile_detected_callback) 
        
        rospy.loginfo("PlayBeepAndCaptureNode is ready.")
        rospy.spin()

    def smile_detected_callback(self, data):
        """
        Callback function for the /smile_detected topic.
        If True (all smiling): Plays beep, captures photo, and publishes photo to /save_photo.
        """
        if data.data:  # If all smiles detected (True)
            rospy.loginfo("All smiles detected! Playing beep and capturing photo...")
            
            # --- Play Beep ---
            try:
                pygame.mixer.music.load(AUDIO_FILE_BEEP) 
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    rospy.sleep(0.1) # Wait for beep to finish
            except pygame.error as e:
                rospy.logerr(f"Error playing beep sound: {e}. Make sure '{AUDIO_FILE_BEEP}' exists.")
            except Exception as e:
                rospy.logerr(f"An unexpected error occurred during beep sound playback: {e}")

            # --- Capture and Publish Photo ---
            if self.camera.isOpened():
                ret, frame = self.camera.read()
                if ret:
                    try:
                        # Convert OpenCV image to ROS Image message
                        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        self.image_pub.publish(ros_image) # This line uses the publisher to send the image!
                        rospy.loginfo("Published captured frame to /save_photo")
                    except Exception as e:
                        rospy.logerr(f"Error converting or publishing image: {e}")
                else:
                    rospy.logwarn("Failed to capture frame from own camera for publishing.")
            else:
                rospy.logwarn("Camera not opened for image capture in PlayBeepAndCaptureNode.")

    def shutdown_hook(self):
        """
        Cleanup method to release camera and quit pygame mixer on node shutdown.
        """
        rospy.loginfo("Shutting down PlayBeepAndCaptureNode. Releasing camera and quitting pygame mixer.")
        if self.camera and self.camera.isOpened():
            self.camera.release()
        if pygame.mixer.get_init():
            pygame.mixer.quit()

if __name__ == '__main__':
    node = None
    try:
        node = PlayBeepAndCaptureNode()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node:
            node.shutdown_hook()