#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import numpy as np
from perception_base.perception_base import PerceptionBase

class FaceDetectUsingNuitrack(PerceptionBase):
    def __init__(self):
        super(FaceDetectUsingNuitrack, self).__init__("social_perception_face_detection")

        self.detected_num = 0

        rospy.loginfo('\033[94m[%s]\033[0m initialze done...'%rospy.get_name())

    def handle_skeleton_detected(self, msg):     
        if self.detected_num != len(msg.skeletons):
            write_data = self.conf_data[self.conf_data.keys()[0]]['data']
            write_data['count'] = len(msg.skeletons)

            if self.detected_num < len(msg.skeletons):
                self.raise_event(self.conf_data.keys()[0], event='face_detected', data=write_data)
            else:
                self.raise_event(self.conf_data.keys()[0], event='face_disappeared', data=write_data)
            
            self.detected_num = len(msg.skeletons) 

        dist_array = []
        for i in range(self.detected_num):
            skeleton = msg.skeletons[i]
            head_point = skeleton.joint_pos[skeleton.joints.index('joint_head')]

            point = np.array([head_point.x, head_point.y, head_point.z])
            dist_array.append(np.linalg.norm(point))

        if self.detected_num > 0:
            near_index = dist_array.index(min(dist_array))
            near_point = msg.skeletons[near_index].joint_pos[msg.skeletons[near_index].joints.index('joint_head')]

            write_data = self.conf_data[self.conf_data.keys()[0]]['data']
            write_data['count'] = 1
            self.save_to_memory(self.conf_data.keys()[0], data=write_data)

            write_data = self.conf_data[self.conf_data.keys()[1]]['data']
            write_data['name'] = 'person'
            write_data['description'] = 'nearest_person'
            write_data['xyz'] = [near_point.z / 1000.0, -1.0 * near_point.x / 1000.0, near_point.y / 1000.0]
            write_data['rpy'] = [0.0, 0.0, 0.0]
            write_data['frame_id'] = 'nuitrack_link'
            self.save_to_memory(self.conf_data.keys()[1], data=write_data)

        else:
            write_data = self.conf_data[self.conf_data.keys()[0]]['data']
            write_data['count'] = 0
            self.save_to_memory(self.conf_data.keys()[0], data=write_data)


if __name__ == '__main__':
    m = FaceDetectUsingNuitrack()
    rospy.spin()