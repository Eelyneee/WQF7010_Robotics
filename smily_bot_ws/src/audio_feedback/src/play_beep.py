#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 

import os
import pygame.mixer

AUDIO_FILE_BEEP = os.path.join(os.path.dirname(__file__), "..", "audio", "beep-25.mp3")

class PlayBeepAndCaptureNode:
    def __init__(self):
        rospy.init_node('play_beep_and_capture', anonymous=True)

        try:
            pygame.mixer.init()
            pygame.mixer.music.set_volume(0.7) 
            rospy.loginfo("Pygame mixer initialized for PlayBeepAndCaptureNode.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Pygame mixer: {e}")
            rospy.signal_shutdown("Could not initialize sound system.")
            return
        
        self.bridge = CvBridge()
        self.last_camera_frame = None 
        rospy.loginfo("PlayBeepAndCaptureNode: Ready to receive camera frames from /usb_cam/image_raw.")

        self.image_pub = rospy.Publisher('/save_photo', Image, queue_size=1) 
        rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback)
        
        rospy.Subscriber("/smile_detected", Bool, self.smile_detected_callback) 
        
        rospy.loginfo("PlayBeepAndCaptureNode is ready.")
        rospy.spin()

    def camera_callback(self, data):
        """
        Callback function for the live camera feed (/usb_cam/image_raw).
        Stores the most recently received camera frame.
        """
        try:
            self.last_camera_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting live camera frame: {e}")
            self.last_camera_frame = None

    def smile_detected_callback(self, data):
        """
        Callback function for the /smile_detected topic.
        If True (all smiling): Plays beep, and publishes the latest captured photo to /save_photo.
        """
        if data.data:  # If all smiles detected (True)
            rospy.loginfo("All smiles detected! Playing beep and attempting to publish photo...")

            try:
                pygame.mixer.music.load(AUDIO_FILE_BEEP) 
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    rospy.sleep(0.1)
            except pygame.error as e:
                rospy.logerr(f"Error playing beep sound: {e}. Make sure '{AUDIO_FILE_BEEP}' exists.")
            except Exception as e:
                rospy.logerr(f"An unexpected error occurred during beep sound playback: {e}")

            if self.last_camera_frame is not None:
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(self.last_camera_frame, "bgr8")
                    self.image_pub.publish(ros_image) # Publish the image
                    rospy.loginfo("Published latest captured frame to /save_photo")
                except Exception as e:
                    rospy.logerr(f"Error converting or publishing image: {e}")
            else:
                rospy.logwarn("No camera frame available to publish when smile was detected. Make sure /usb_cam/image_raw is publishing.")

    def shutdown_hook(self):
        """
        Cleanup method to quit pygame mixer on node shutdown.
        No camera to release, as it's handled by another node (like usb_cam_node).
        """
        rospy.loginfo("Shutting down PlayBeepAndCaptureNode. Quitting pygame mixer.")
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