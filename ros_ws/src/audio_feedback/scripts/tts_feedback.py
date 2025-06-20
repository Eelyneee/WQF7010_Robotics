#!/usr/bin/env python3
import rospy
import pyttsx3
import threading
from std_msgs.msg import String

class TTSFeedback:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('tts_feedback', anonymous=True)
        
        # Initialize TTS engine
        self.tts_engine = pyttsx3.init()
        self.setup_voice()
        
        # Subscribe to text input topic
        self.text_subscriber = rospy.Subscriber('/say_text', String, self.speak_callback)
        
        rospy.loginfo("🔊 TTS Feedback node initialized and ready!")
        print("🔊 TTS Feedback node is listening on /say_text topic")
        
    def setup_voice(self):
        """Configure TTS engine settings"""
        try:
            # Set speech rate (words per minute)
            self.tts_engine.setProperty('rate', 150)
            
            # Set volume (0.0 to 1.0)
            self.tts_engine.setProperty('volume', 0.9)
            
            # Get available voices and set to English if available
            voices = self.tts_engine.getProperty('voices')
            if voices:
                for voice in voices:
                    if 'english' in voice.name.lower() or 'en' in voice.id.lower():
                        self.tts_engine.setProperty('voice', voice.id)
                        break
                        
        except Exception as e:
            rospy.logwarn(f"TTS setup warning: {e}")
    
    def speak_callback(self, msg):
        """Callback function to handle incoming text messages"""
        text_to_speak = msg.data
        rospy.loginfo(f"🔊 Speaking: {text_to_speak}")
        
        # Use threading to prevent blocking
        speak_thread = threading.Thread(target=self.speak_text, args=(text_to_speak,))
        speak_thread.daemon = True
        speak_thread.start()
    
    def speak_text(self, text):
        """Convert text to speech"""
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            rospy.logerr(f"TTS Error: {e}")
    
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        tts_node = TTSFeedback()
        tts_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TTS Feedback node stopped.")