#!/usr/bin/env python3
import rospy
import speech_recognition as sr
import threading
from std_msgs.msg import String
import time

class VoiceListener:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('voice_listener', anonymous=True)
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Publishers
        self.voice_command_pub = rospy.Publisher('/voice_command', String, queue_size=10)
        self.say_text_pub = rospy.Publisher('/say_text', String, queue_size=10)
        
        # Robot responses
        self.responses = {
            'hello': "Hello there! Nice to meet you!",
            'hi': "Hi! How can I help you today?",
            'how are you': "I'm doing great! Thanks for asking.",
            'what is your name': "I'm your friendly robot assistant!",
            'take photo': "Say cheese! Taking a photo now!",
            'smile': "I love seeing smiles! Keep it up!",
            'goodbye': "Goodbye! Have a wonderful day!",
            'bye': "See you later!",
            'thank you': "You're very welcome!",
            'thanks': "My pleasure to help!",
            'stop listening': "I'll stop listening now. Goodbye!",
            'quit': "Goodbye! It was nice talking with you!"
        }
        
        # Setup microphone
        self.setup_microphone()
        
        rospy.loginfo("🎤 Voice Listener node initialized!")
        self.say_text_pub.publish(String(data="Voice listener ready! Say something to me!"))
        
    def setup_microphone(self):
        """Setup and calibrate microphone"""
        try:
            rospy.loginfo("🎤 Setting up microphone...")
            with self.microphone as source:
                # Adjust for ambient noise
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
                rospy.loginfo("🎤 Microphone calibrated for ambient noise")
        except Exception as e:
            rospy.logerr(f"Microphone setup error: {e}")
    
    def listen_for_speech(self):
        """Listen for speech input"""
        try:
            with self.microphone as source:
                rospy.loginfo("🎤 Listening...")
                # Listen for audio with timeout
                audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)
                
            rospy.loginfo("🎤 Processing speech...")
            
            # Recognize speech using Google Speech Recognition
            try:
                text = self.recognizer.recognize_google(audio)
                rospy.loginfo(f"🎤 You said: {text}")
                return text.lower()
            except sr.UnknownValueError:
                rospy.logwarn("🎤 Could not understand audio")
                return None
            except sr.RequestError as e:
                rospy.logerr(f"🎤 Speech recognition error: {e}")
                return None
                
        except sr.WaitTimeoutError:
            # Timeout is normal, just return None
            return None
        except Exception as e:
            rospy.logerr(f"🎤 Listen error: {e}")
            return None
    
    def get_response(self, user_input):
        """Generate robot response based on user input"""
        user_input = user_input.lower().strip()
        
        # Check for exact matches first
        if user_input in self.responses:
            return self.responses[user_input]
        
        # Check for partial matches
        for key, response in self.responses.items():
            if key in user_input:
                return response
        
        # Default response for unknown input
        return f"You said: {user_input}. That's interesting!"
    
    def run(self):
        """Main listening loop"""
        rospy.loginfo("🎤 Starting voice recognition loop...")
        
        while not rospy.is_shutdown():
            try:
                # Listen for speech
                speech_text = self.listen_for_speech()
                
                if speech_text:
                    # Publish the raw voice command
                    self.voice_command_pub.publish(String(data=speech_text))
                    
                    # Check for quit commands
                    if any(quit_word in speech_text for quit_word in ['quit', 'stop listening', 'goodbye']):
                        response = self.get_response(speech_text)
                        self.say_text_pub.publish(String(data=response))
                        rospy.sleep(3)  # Wait for TTS to finish
                        break
                    
                    # Generate and speak response
                    response = self.get_response(speech_text)
                    self.say_text_pub.publish(String(data=response))
                
                # Small delay to prevent overwhelming the system
                rospy.sleep(0.1)
                
            except KeyboardInterrupt:
                rospy.loginfo("🎤 Voice listener stopped by user")
                self.say_text_pub.publish(String(data="Goodbye!"))
                break
            except Exception as e:
                rospy.logerr(f"🎤 Main loop error: {e}")
                rospy.sleep(1)

if __name__ == '__main__':
    try:
        voice_node = VoiceListener()
        voice_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice Listener node stopped.")