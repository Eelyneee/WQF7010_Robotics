#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gtts import gTTS
import pygame
import os
import time # Import time for delay and file management

class TTSFeedback:
    def __init__(self):
        rospy.init_node('tts_feedback', anonymous=True)
        
        # Initialize pygame mixer
        pygame.mixer.init()

        self.text_subscriber = rospy.Subscriber('/say_text', String, self.text_callback)
        self.status_publisher = rospy.Publisher('/tts_status', String, queue_size=1)

        # Ensure the 'audio' directory exists
        self.audio_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'audio')
        if not os.path.exists(self.audio_dir):
            os.makedirs(self.audio_dir)
            rospy.loginfo(f"Created audio directory: {self.audio_dir}")

        rospy.set_param('tts_busy', False) # Initialize ROS parameter
        self.publish_status("idle") # Publish initial status

        rospy.loginfo("TTS Feedback Node initialized with gTTS.")
        rospy.loginfo("Ensure 'gTTS' and 'pygame' are installed (pip install gTTS pygame).")
        rospy.loginfo("An internet connection is required for gTTS.")


    def publish_status(self, status):
        """Helper to publish TTS status and update ROS param."""
        self.status_publisher.publish(status)
        rospy.set_param('tts_busy', (status == "busy")) 

    def text_callback(self, msg):
        text_to_speak = msg.data
        rospy.loginfo("Received text to speak: '%s'", text_to_speak)

        # Set status to busy before speaking
        self.publish_status("busy") 

        audio_file = os.path.join(self.audio_dir, "speech.mp3") # Define file path

        try:
            # Generate gTTS audio
            tts = gTTS(text=text_to_speak, lang='en', slow=False) # 'en' for English, adjust as needed
            tts.save(audio_file)
            rospy.loginfo("gTTS audio saved to %s", audio_file)

            # Load and play with pygame
            pygame.mixer.music.load(audio_file)
            pygame.mixer.music.play()

            # Wait until playback finishes
            while pygame.mixer.music.get_busy():
                rospy.sleep(3) # Check every 100ms

            rospy.loginfo("Finished playing audio with gTTS.")

        except Exception as e:
            rospy.logerr("Error in TTS playback with gTTS: %s", str(e))
            rospy.logwarn("Ensure you have an internet connection for gTTS to work.")
        finally:
            # Clean up the temporary audio file
            if os.path.exists(audio_file):
                os.remove(audio_file)
                rospy.loginfo("Removed temporary audio file: %s", audio_file)
            
            # Set status to idle after speaking (or if an error occurred)
            self.publish_status("idle")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tts_node = TTSFeedback()
        tts_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TTS Feedback node shut down.")
    except Exception as e:
        rospy.logerr("Failed to start TTS Feedback node: %s", str(e))