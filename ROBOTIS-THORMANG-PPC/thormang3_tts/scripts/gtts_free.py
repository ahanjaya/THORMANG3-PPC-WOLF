#!/usr/bin/env python3

import os
import rospy
import rospkg
import urllib.request
from gtts import gTTS
from std_msgs.msg import String

class TTS:
    def __init__(self):
        rospy.init_node("text_to_speech")
        rospy.loginfo("[TTS] Text to Speech Running")

        rospack           = rospkg.RosPack()
        self.sound_path   = rospack.get_path("thormang3_tts") + "/sounds"

        # Subscriber
        rospy.Subscriber('/robotis/sensor/text_to_speech', String, self.tts_callback)
        rospy.Subscriber('/robotis/sensor/update_tts',     String, self.update_callback)

        # Publisher
        self.tts_status_pub = rospy.Publisher("/robotis/sensor/tts_status", String, queue_size=1)

    def tts_callback(self, msg):
        self.tts_status_pub.publish('parsing text')

        # 1. Parsing text
        ret, mp3_file = self.parse_text(msg.data)

        # 2. Playing mp3_file
        if ret:
            self.playing_mp3(mp3_file)

    def update_callback(self, msg):
        self.tts_status_pub.publish('parsing text')

        # 1. Parsing text
        ret, mp3_file = self.parse_text(msg.data, update=True)

        # 2. Playing mp3_file
        if ret:
            self.playing_mp3(mp3_file)

    def is_connected(self):
        try:
            urllib.request.urlopen('http://google.com')
            return True
        except:
            return False

    def parse_text(self, text, update=False):
        raw_text = text.split(') ')
        
        # check raw text format
        if len(raw_text) == 2:
            rospy.loginfo("[TTS] Received data: {}".format(raw_text))
            mp3_name  = raw_text[0]
            text_file = raw_text[1]

            # check mp3 file
            mp3_file = "{}/{}.mp3".format(self.sound_path, mp3_name)

            # if file exist
            if os.path.exists(mp3_file) and not update:
                rospy.loginfo("[TTS] Found file: {}".format(mp3_file))
                return True, mp3_file

            # if file not found
            else:
                # check internet connection
                if self.is_connected():
                    rospy.loginfo("[TTS] Generating mp3 file: {}".format(mp3_file))
                    tts = gTTS(text=text_file, lang='en-us')
                    tts.save(mp3_file)
                    return True, mp3_file

                # please fix internet connection
                else:
                    rospy.logwarn("[TTS] Please fix internet connection")
        else:
            rospy.logwarn("[TTS] Received data: {}".format(raw_text))
            rospy.logwarn("[TTS] Invalid format, please use number format and followed by text")
            rospy.logwarn("[TTS] Example: '1) I love Jacky'")
            return False, None

    def playing_mp3(self, path):
        self.tts_status_pub.publish('start talking')
        os.system("mpg321 {}".format(path))
        
        rospy.loginfo("[TTS] Finished playing: {}".format(path))
        self.tts_status_pub.publish('finish talking')
  
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    tts = TTS()
    tts.run()