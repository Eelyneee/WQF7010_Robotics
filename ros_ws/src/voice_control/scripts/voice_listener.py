#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import threading
import time

class VoiceListener:
    def __init__(self):
        rospy.init_node('voice_listener', anonymous=True)

        self.command_publisher = rospy.Publisher('/voice_command', String, queue_size=10)
        self.state_subscriber = rospy.Subscriber('/milo_conversation_state', String, self.state_callback)

        self.wake_words = ["hi milo", "milo wake up", "milo are you there"]

        self.conversation_mode_active = False 

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.mode_lock = threading.Lock() 

        rospy.loginfo("Voice Listener Node initialized. Starting microphone listening thread.")
        rospy.loginfo("Initially waiting for wake word: '%s'", ", ".join(self.wake_words))

        self.listening_thread = threading.Thread(target=self.listen_loop)
        self.listening_thread.daemon = True 
        self.listening_thread.start()

    def state_callback(self, msg):
        """Callback for conversation state updates from interaction_manager."""
        with self.mode_lock:
            new_state = msg.data
            if new_state == "active" and not self.conversation_mode_active:
                self.conversation_mode_active = True
                rospy.loginfo("Voice Listener: Conversation mode set to ACTIVE by Interaction Manager.")
            elif new_state == "passive" and self.conversation_mode_active:
                self.conversation_mode_active = False
                rospy.loginfo("Voice Listener: Conversation mode set to PASSIVE by Interaction Manager. Waiting for wake word.")

    def process_command(self, text_input):
        """
        Processes the input text to detect the wake word or subsequent commands.
        Publishes commands to /voice_command.
        """
        with self.mode_lock:
            text_input_lower = text_input.lower().strip()
            rospy.loginfo("Processing recognized text: '%s'", text_input_lower)

            if not self.conversation_mode_active:
                if any(wake_word in text_input_lower for wake_word in self.wake_words):
                    rospy.loginfo("Wake word detected. Publishing 'robot_greet'.")
                    self.command_publisher.publish("robot_greet")
                else:
                    rospy.loginfo("Passively listening. Still waiting for wake word: '%s'...", ", ".join(self.wake_words))
            else:
                command_text = text_input_lower
                for ww in self.wake_words:
                    command_text = command_text.replace(ww, "").strip()

                if any(phrase in command_text for phrase in ["take a photo", "take a picture", "let's take a picture", "click the shutter"]):
                    self.command_publisher.publish("take_photo")
                elif any(phrase in command_text for phrase in ["show me the last photo", "view the picture", "can i see that", "review photos"]):
                    self.command_publisher.publish("view_photo")
                elif "save this photo" in command_text:
                    self.command_publisher.publish("save_photo")
                elif any(phrase in command_text for phrase in ["goodbye milo", "stop listening", "thanks milo", "that's all for now", "exit conversation"]):
                    self.command_publisher.publish("goodbye_command") 
                elif any(phrase in command_text for phrase in ["check my smile", "am i smiling enough", "can you detect smiles"]):
                    self.command_publisher.publish("check_smile")
                elif any(phrase in command_text for phrase in ["how are we looking", "check our expressions"]):
                    self.command_publisher.publish("check_expressions")
                elif any(phrase in command_text for phrase in ["suggest a pose", "give us a pose idea", "help us pose"]):
                    self.command_publisher.publish("suggest_pose")
                elif any(phrase in command_text for phrase in ["are we all in the frame", "is everyone visible", "adjust for the group"]):
                    self.command_publisher.publish("check_framing")
                elif any(phrase in command_text for phrase in ["what can you do", "help", "tell me your commands"]):
                    self.command_publisher.publish("help_command")
                elif "retry" in command_text:
                    self.command_publisher.publish("retry")
                elif "repeat" in command_text or "say that again" in command_text:
                    self.command_publisher.publish("repeat")
                else:
                    self.command_publisher.publish("unrecognized_command")

    def listen_loop(self):
        """
        Continuously listens to the microphone for speech, pausing when TTS is busy.
        Runs in a separate thread.
        """
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Microphone calibrated for ambient noise.")

            while not rospy.is_shutdown():
                if rospy.has_param('tts_busy'):
                    while rospy.get_param('tts_busy', False) and not rospy.is_shutdown():
                        rospy.loginfo_once("Voice Listener: TTS is busy. Pausing microphone input.")
                        rospy.sleep(5)
                
                rospy.loginfo_once("Voice Listener: Microphone listening for speech...")

                try:
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)

                    rospy.loginfo("Microphone: Recognizing speech...")
                    text = self.recognizer.recognize_google(audio)
                    self.process_command(text)

                except sr.WaitTimeoutError:
                    pass
                except sr.UnknownValueError:
                    rospy.logwarn("Microphone: Could not understand audio.")
                    with self.mode_lock:
                        if self.conversation_mode_active and not rospy.get_param('tts_busy', False):
                            self.command_publisher.publish("unrecognized_command")
                except sr.RequestError as e:
                    rospy.logerr("Microphone: Could not request results from Google Speech Recognition service; %s", e)
                    rospy.logwarn("Check your internet connection or Google API limits.")
                except Exception as e:
                    rospy.logerr("Microphone: An unexpected error occurred: %s", str(e))

                rospy.sleep(3)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        listener = VoiceListener()
        listener.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice Listener node shut down cleanly.")
    except Exception as e:
        rospy.logerr("Failed to start Voice Listener node: %s", str(e))