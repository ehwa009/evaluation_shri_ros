#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from perception_base.perception_base import PerceptionBase
from std_msgs.msg import Bool


class SpeechRecognizer(PerceptionBase):
    def __init__(self):
        super(SpeechRecognizer, self).__init__("social_perception_speech_recognizer")
        self.pub_enable_speech = rospy.Publisher('enable_recognition', Bool, queue_size=5)
        rospy.loginfo('\033[94m[%s]\033[0m initialze done...'%rospy.get_name())

    def handle_recognized_word(self, msg):
        write_data = self.conf_data[self.conf_data.keys()[0]]['data']

        write_data['recognized_word'] = msg.recognized_word
        write_data['confidence'] = msg.confidence

        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
        self.raise_event(self.conf_data.keys()[0], event='speech_recognized', data=write_data)        

    def handle_start_perception(self, msg):
        self.pub_enable_speech.publish(True)
        super(SpeechRecognizer, self).handle_start_perception(msg)

    def handle_stop_perception(self, msg):
        self.pub_enable_speech.publish(False)
        rospy.sleep(0.2)
        super(SpeechRecognizer, self).handle_stop_perception(msg)


if __name__ == '__main__':
    m = SpeechRecognizer()
    rospy.spin()