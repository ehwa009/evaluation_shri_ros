#!/usr/bin/env python
# -*- encoding: utf8 -*-

import os
import rospy
import rospkg
import re

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from mind_msgs.msg import RaisingEvents
from std_msgs.msg import Int16

class CommandButtons(Plugin):
    def __init__(self, context):
        super(CommandButtons, self).__init__(context)
        self.setObjectName('CommandButtons')

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_shri_command_buttons'), 'resource', 'command_buttons.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('CommandButtonsUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        self.pub_events = rospy.Publisher('raising_events', RaisingEvents, queue_size=1)
        self.pub_select_scenario = rospy.Publisher('/select_evaluation_scenario', Int16, queue_size=1)

        self._widget.buttonSelect0.clicked.connect(self.handle_select_scenario)
        self._widget.buttonSelect1.clicked.connect(self.handle_select_scenario)
        self._widget.buttonSelect2.clicked.connect(self.handle_select_scenario)
        self._widget.buttonSelect3.clicked.connect(self.handle_select_scenario)

        self._widget.buttonStart.clicked.connect(self.handle_start_scenario)
        self._widget.buttonStart.setEnabled(False)
        self.scenario_selected = False
        self._widget.textStatus.setText("Ready...")

    def handle_select_scenario(self):
        button = self.sender()
        cmd_msg = re.findall('\[(.+)\]', button.text())[0]

        if self.scenario_selected:
            self._widget.buttonSelect0.setEnabled(True)
            self._widget.buttonSelect1.setEnabled(True)
            self._widget.buttonSelect2.setEnabled(True)
            self._widget.buttonSelect3.setEnabled(True)
            self._widget.textStatus.setText("Ready...")
            self._widget.buttonStart.setEnabled(False)
            self.scenario_selected = False
            return

        if cmd_msg.lower() == 'neutral':
            self._widget.textStatus.setText("Select %s"%button.text())
            self._widget.buttonSelect1.setEnabled(False)
            self._widget.buttonSelect2.setEnabled(False)
            self._widget.buttonSelect3.setEnabled(False)
            self._widget.buttonStart.setEnabled(True)
            self.pub_select_scenario.publish(0)
            self.scenario_selected = True

        elif cmd_msg.lower() == 'leaning forward':
            self._widget.textStatus.setText("Select %s"%button.text())
            self._widget.buttonSelect0.setEnabled(False)
            self._widget.buttonSelect2.setEnabled(False)
            self._widget.buttonSelect3.setEnabled(False)
            self._widget.buttonStart.setEnabled(True)
            self.pub_select_scenario.publish(1)
            self.scenario_selected = True

        elif cmd_msg.lower() == 'self disclosure':
            self._widget.textStatus.setText("Select %s"%button.text())
            self._widget.buttonSelect0.setEnabled(False)
            self._widget.buttonSelect1.setEnabled(False)
            self._widget.buttonSelect3.setEnabled(False)
            self._widget.buttonStart.setEnabled(True)
            self.pub_select_scenario.publish(2)
            self.scenario_selected = True

        elif cmd_msg.lower() == 'voice pitch':
            self._widget.textStatus.setText("Select %s"%button.text())
            self._widget.buttonSelect0.setEnabled(False)
            self._widget.buttonSelect1.setEnabled(False)
            self._widget.buttonSelect2.setEnabled(False)
            self._widget.buttonStart.setEnabled(True)
            self.pub_select_scenario.publish(3)
            self.scenario_selected = True

    def handle_start_scenario(self):
        msg = RaisingEvents()
        msg.header.stamp = rospy.Time.now()
        msg.events.append('human_appeared')

        self.pub_events.publish(msg)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass