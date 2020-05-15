#!/usr/bin/env python3

import os
import rospy
import rospkg
import pyttsx3
from gtts import gTTS
from std_msgs.msg import String

class TTS:
    def __init__(self):
        rospy.init_node("text_to_speech")
        rospy.loginfo("[TTS] Text to Speech Running")

        rospack           = rospkg.RosPack()
        self.sound_path   = rospack.get_path("thormang3_sensors") + "/scripts/tts.mp3"
        self.use_gTTs     = True

        # pyttsx3 method
        self.engine       = pyttsx3.init()
        self.engine.setProperty('voice', 'english-us')
        rate = self.engine.getProperty('rate')   # getting details of current speaking rate
        self.engine.setProperty('rate', 150)     # setting up new voice rate
        
        # Subscriber
        rospy.Subscriber('/robotis/sensor/text_to_speech', String, self.tts_callback)

    def tts_callback(self, msg):
        text = msg.data
        
        if self.use_gTTs:
            tts  = gTTS(text=text, lang='en-us')
            tts.save(self.sound_path)
            os.system("mpg321 {}".format(self.sound_path))
        else:
            self.engine.say(text)
            self.engine.runAndWait()

  
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    tts = TTS()
    tts.run()