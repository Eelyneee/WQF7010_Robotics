#!/usr/bin/env python3
import rospy
import os
import tempfile
import threading
from gtts import gTTS
from std_msgs.msg import String
import subprocess

class RobustGoogleTTSFeedback:
    def __init__(self):
        rospy.init_node('robust_google_tts_feedback', anonymous=True)
        
        # Find working audio player
        self.audio_player = self.find_audio_player()
        if not self.audio_player:
            rospy.logerr("❌ No working audio player found!")
            return
        
        rospy.loginfo(f"✅ Using audio player: {self.audio_player}")
        
        self.text_subscriber = rospy.Subscriber('/say_text', String, self.speak_callback)
        self.language = 'en'
        self.slow_speech = False
        
        rospy.loginfo("🔊 Robust Google TTS Feedback node ready!")
        
    def find_audio_player(self):
        """Find a working audio player"""
        players = ['mpg321', 'mpg123', 'ffplay', 'cvlc']
        
        for player in players:
            try:
                result = subprocess.run([player, '--version'], 
                                      capture_output=True, timeout=5)
                if result.returncode == 0:
                    rospy.loginfo(f"✅ Found working audio player: {player}")
                    return player
            except:
                continue
        
        rospy.logerr("❌ No audio player found. Install with:")
        rospy.logerr("sudo apt-get install mpg321 mpg123 ffmpeg")
        return None
    
    def speak_callback(self, msg):
        if not self.audio_player:
            rospy.logerr("❌ No audio player available")
            return
            
        text_to_speak = msg.data
        rospy.loginfo(f"🔊 Speaking: {text_to_speak}")
        
        speak_thread = threading.Thread(target=self.speak_text, args=(text_to_speak,))
        speak_thread.daemon = True
        speak_thread.start()
    
    def speak_text(self, text):
        try:
            tts = gTTS(text=text, lang=self.language, slow=self.slow_speech)
            
            with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as tmp_file:
                temp_filename = tmp_file.name
                tts.save(temp_filename)
                
                # Use the working audio player
                cmd = [self.audio_player]
                if self.audio_player == 'ffplay':
                    cmd.extend(['-nodisp', '-autoexit'])
                elif self.audio_player == 'cvlc':
                    cmd.extend(['--intf', 'dummy', '--play-and-exit'])
                
                cmd.append(temp_filename)
                
                try:
                    result = subprocess.run(cmd, capture_output=True, timeout=30)
                    if result.returncode != 0:
                        rospy.logwarn(f"Audio player warning: {result.stderr.decode()}")
                except Exception as e:
                    rospy.logerr(f"Audio playback error: {e}")
                finally:
                    try:
                        os.unlink(temp_filename)
                    except:
                        pass
                        
        except Exception as e:
            rospy.logerr(f"TTS Error: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tts_node = RobustGoogleTTSFeedback()
        tts_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robust Google TTS node stopped.")