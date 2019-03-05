#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import actionlib

from std_msgs.msg import Bool
from polly_speech.msg import SpeechAction, SpeechGoal
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback

from langdetect import detect

class EvaluationRenderSpeech:
    def __init__(self):
        self.is_multi_lang = rospy.get_param('~use_multi_speech')
        self.lang = ['en-US', 'ko-KR']
        self.detect_eng = ['en', 'no', 'so', 'hr', 'cs']
        self.detect_kor = ['ko']

        # start two speech client
        if self.is_multi_lang:
            self.internal_client_1, self.internal_client_2 = ( actionlib.SimpleActionClient('internal_speech_%s'%l, SpeechAction) for l in self.lang )
            rospy.loginfo('Wait for bringup internal speech server...')
            self.internal_client_1.wait_for_server()
            self.internal_client_2.wait_for_server()
        else:
            self.internal_client = actionlib.SimpleActionClient('internal_speech', SpeechAction)
            rospy.loginfo('Wait for bringup internal speech server...')
            self.internal_client.wait_for_server()

        self.pub_status = rospy.Publisher('u_is_speaking', Bool, queue_size=10)
        rospy.Subscriber('u_initialized', Bool, self.handle_initialized)
        self.is_speaking = False
        self.u_initialized = True

        self.server = actionlib.SimpleActionServer('render_speech', RenderItemAction, self.execute_callback, False)
        self.server.start()
        rospy.loginfo('%s initialized...'%rospy.get_name())

    def handle_initialized(self, msg):
        self.u_initialized = msg.data

    def func_feedback(self, feedback):
        self.is_speaking = feedback.is_speaking

    def func_done(self, state, result):
        self.is_speaking = False

    def execute_callback(self, goal):
        rospy.loginfo('%s rendering requested [%s]...' % (rospy.get_name(), goal.data))

        result = RenderItemResult()
        feedback = RenderItemFeedback()
        result.result = True

        if not self.u_initialized:
            rospy.sleep(0.2)
            result.result = True
            self.server.set_succeeded(result)
            return

        self.is_speaking = True
        self.pub_status.publish(True)
        feedback.is_rendering = True

        internal_goal = SpeechGoal()
        internal_goal.text = goal.data
        
        if self.is_multi_lang:
            lang = detect(goal.data.decode('utf-8'))
            rospy.loginfo('Detected lang: %s', lang)
            if lang in self.detect_eng:
                self.internal_client_1.send_goal(internal_goal, done_cb=self.func_done, feedback_cb=self.func_feedback)
            elif lang in self.detect_kor:
                self.internal_client_2.send_goal(internal_goal, done_cb=self.func_done, feedback_cb=self.func_feedback)
        else:
            self.internal_client.send_goal(internal_goal, done_cb=self.func_done, feedback_cb=self.func_feedback)

        while not rospy.is_shutdown() and self.is_speaking:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                result.result = False
                break

            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)

        if result.result:
            self.server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('evaluation_render_speech', anonymous=False)
    tts = EvaluationRenderSpeech()
    rospy.spin()