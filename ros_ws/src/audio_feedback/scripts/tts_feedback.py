#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os
import pygame # pygame is used for playing audio files

# Although the prompt mentioned Google TTS directly, for a simple ROS node that can run
# easily without complex API key management, gTTS is a convenient library.
# It uses Google's Text-to-Speech API in the backend.
# You might need to install it: pip install gTTS pygame
from gtts import gTTS

class TTSFeedback:
    """
    ROS Node for converting text to speech and playing it.

    This node subscribes to the '/say_text' topic, takes the received string,
    converts it into speech using Google Text-to-Speech (via gTTS library),
    and plays the generated audio file.
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tts_feedback', anonymous=True)

        # Initialize pygame mixer for audio playback
        # Adjust frequency and buffer if you experience audio issues
        pygame.mixer.init(frequency=24000, size=-16, channels=2, buffer=512)

        # Create a subscriber to the /say_text topic
        # When a message is received, the self.text_callback method is called
        self.text_subscriber = rospy.Subscriber('/say_text', String, self.text_callback)

        # Path for temporary audio files
        self.audio_file_path = "/tmp/tts_output.mp3"

        rospy.loginfo("TTS Feedback Node initialized. Waiting for text on /say_text topic.")

    def text_callback(self, msg):
        """
        Callback function for when a message is received on the /say_text topic.
        Converts the received text to speech and plays it.
        """
        text_to_speak = msg.data
        rospy.loginfo("Received text to speak: '%s'", text_to_speak)

        try:
            # Create a gTTS object
            # lang='en' specifies the language (English)
            tts = gTTS(text=text_to_speak, lang='en', slow=False)

            # Save the audio to a temporary MP3 file
            tts.save(self.audio_file_path)
            rospy.loginfo("Generated speech audio saved to: %s", self.audio_file_path)

            # Load and play the audio file using pygame
            if os.path.exists(self.audio_file_path):
                pygame.mixer.music.load(self.audio_file_path)
                pygame.mixer.music.play()
                # Wait for the audio to finish playing
                while pygame.mixer.music.get_busy():
                    rospy.sleep(0.1)
                rospy.loginfo("Finished playing audio.")
            else:
                rospy.logwarn("Audio file not found at %s", self.audio_file_path)

        except Exception as e:
            rospy.logerr("Error in TTS or audio playback: %s", str(e))
            rospy.logwarn("Ensure you have 'gTTS' and 'pygame' installed (pip install gTTS pygame) "
                          "and an audio output device configured.")

    def run(self):
        """
        Keeps the node running until shutdown.
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        tts_node = TTSFeedback()
        tts_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TTS Feedback node shut down.")
    finally:
        # Clean up temporary audio file if it exists
        if os.path.exists("/tmp/tts_output.mp3"):
            os.remove("/tmp/tts_output.mp3")
            rospy.loginfo("Cleaned up temporary audio file.")
        pygame.mixer.quit() # Quit pygame mixer
