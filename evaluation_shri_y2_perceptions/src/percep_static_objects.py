#!/usr/bin/python
#-*- encoding: utf8 -*-

import os
import yaml
import rospy
from perception_base.perception_base import PerceptionBase

from std_msgs.msg import Float64

class StaticObjectInfoPublisher(PerceptionBase):
    def __init__(self):
        super(StaticObjectInfoPublisher, self).__init__("static_object_info_publisher")

        refresh_period = rospy.get_param('~refresh_period', 1.0)
        self.object_list_file = rospy.get_param('~object_list_file', '')

        rospy.Timer(rospy.Duration(refresh_period), self.handle_periodic_publish)
        rospy.loginfo('[%s] initialze done...'%rospy.get_name())
        rospy.spin()

    def handle_periodic_publish(self, event):
        if self.object_list_file == '':
            rospy.logwarn('Empty objects_info. Please check the information...')
            return

        with open(os.path.abspath(self.object_list_file)) as f:
            object_list = yaml.load(f.read())

        for k, v in object_list.items():
            write_data = self.conf_data['objects']['data']
            write_data['name'] = k
            write_data.update(v)

            self.save_to_memory('objects', data=write_data)

        write_info_data = self.conf_data['objects_info']['data']
        write_info_data['num_of_objects'] = len(object_list)
        self.save_to_memory('objects_info', data=write_info_data)

if __name__ == '__main__':
    m = StaticObjectInfoPublisher()
