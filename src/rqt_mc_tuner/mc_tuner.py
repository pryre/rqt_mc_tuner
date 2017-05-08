import os
import math
import rospkg
import rospy
import tf

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

import mavros
from mavros import command
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import State
from mavros.param import *

class MCTuner(Plugin):
	def __init__(self, context):
		super(MCTuner, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('MCTuner')
		rp = rospkg.RosPack()

		# Process standalone plugin command-line arguments
		#from argparse import ArgumentParser
		#parser = ArgumentParser()
		# Add argument(s) to the parser.
		#parser.add_argument("-q", "--quiet", action="store_true",
		#              dest="quiet",
		#              help="Put plugin in silent mode")
		#args, unknowns = parser.parse_known_args(context.argv())
		#if not args.quiet:
		#    print 'arguments: ', args
		#    print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		ui_file = os.path.join(rp.get_path('rqt_mc_tuner'), 'resource', 'MCTuner.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('MCTunerUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		self._widget.button_refresh_topics.clicked.connect(self.button_refresh_topics_pressed)
		self._widget.button_halt.clicked.connect(self.button_halt_pressed)
		self._widget.button_arm.clicked.connect(self.button_arm_pressed)
		self._widget.button_disarm.clicked.connect(self.button_disarm_pressed)
		self._widget.button_gains_read.clicked.connect(self.button_gains_read_pressed)
		self._widget.button_gains_set.clicked.connect(self.button_gains_set_pressed)
		self._widget.button_run_test_rates.clicked.connect(self.button_run_test_rates_pressed)
		self._widget.button_run_test_attitude.clicked.connect(self.button_run_test_attitude_pressed)
		self._widget.button_write_parameters.clicked.connect(self.button_write_parameters_pressed)

		self.test_state = 0 #TODO: Use enum (0: no throttle, 1: rates test, 2: attitude test)

		# TODO: Dropdown box to select setpoint output
		self.pub_sp = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
		self.sub_state = rospy.Subscriber("/mavros/state", State, self.callback_state)

		mavros.set_namespace()

		self.loop_rate = 25.0 #Hz
		self.timer_setpoint = rospy.Timer(rospy.Duration(1 / self.loop_rate), self.output_target_attitude)

		# TODO: Attitude target publisher
			# TODO: Output this constantly on a loop?
			# TODO: Make a counter/something to output the min-max setpoints for tests
			# TODO: Use tf to convert the min-max attitude values to quaternions
		# TODO: Have a service to arm/disarm (and call this on halt
		# TODO: Have a service to read and set parameters
		# TODO: Have a service to send the EEPROM write command

	def shutdown_plugin(self):
		# TODO unregister all publishers here
		self.timer_setpoint.shutdown()

		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def callback_state(self, data):
		self._widget.button_refresh_topics.setEnabled(True)
		self._widget.button_halt.setEnabled(True)
		self._widget.button_arm.setEnabled(True)
		self._widget.button_disarm.setEnabled(True)
		self._widget.button_gains_read.setEnabled(True)
		self._widget.button_gains_set.setEnabled(True)
		self._widget.button_run_test_rates.setEnabled(True)
		self._widget.button_run_test_attitude.setEnabled(True)
		self._widget.button_write_parameters.setEnabled(True)

		self._widget.spinbox_p_gain.setEnabled(True)
		self._widget.spinbox_i_gain.setEnabled(True)
		self._widget.spinbox_d_gain.setEnabled(True)
		self._widget.spinbox_setpoint_attitude.setEnabled(True)
		self._widget.spinbox_setpoint_rate.setEnabled(True)

		self._widget.slider_throttle.setEnabled(True)

	def do_arm(self, state):
		try:
			command.arming(value=state)
		except rospy.ServiceException as ex:
			rospy.logerr(ex)

	def button_refresh_topics_pressed(self):
		rospy.loginfo("Refresh topics button pressed!")

	def button_halt_pressed(self):
		rospy.loginfo("Halt button pressed!")
		self.test_state = 0

	def button_arm_pressed(self):
		self.do_arm(True)

	def button_disarm_pressed(self):
		self.do_arm(False)

	def button_gains_read_pressed(self):
		try:
			self._widget.spinbox_p_gain.setValue(param_get("PID_ROLL_R_P"))
			self._widget.spinbox_i_gain.setValue(param_get("PID_ROLL_R_I"))
			self._widget.spinbox_d_gain.setValue(param_get("PID_ROLL_R_D"))
		except rospy.ServiceException as ex:
			rospy.logerr(ex)

	def button_gains_set_pressed(self):
		try:
			rospy.loginfo(param_set("PID_ROLL_R_P", self._widget.spinbox_p_gain.value()))
			rospy.loginfo(param_set("PID_ROLL_R_I", self._widget.spinbox_i_gain.value()))
			rospy.loginfo(param_set("PID_ROLL_R_D", self._widget.spinbox_d_gain.value()))
		except rospy.ServiceException as ex:
			rospy.logerr(ex)

	def button_run_test_rates_pressed(self):
		rospy.loginfo("Run rates button pressed!")
		self.test_state = 1

	def button_run_test_attitude_pressed(self):
		rospy.loginfo("Run attitude button pressed!")
		self.test_state = 2

	def button_write_parameters_pressed(self):
		rospy.loginfo("DEBUG: Write EEPROM button pressed!")

	def output_target_attitude(self, timer_event):
		at_out = AttitudeTarget()

		if(self.test_state == 0):
			at_out.thrust = 0.0

			at_out.body_rate.x = 0.0
			at_out.body_rate.y = 0.0
			at_out.body_rate.z = 0.0

			at_out.orientation.w = 1.0
			at_out.orientation.x = 0.0
			at_out.orientation.y = 0.0
			at_out.orientation.z = 0.0

			at_out.type_mask = AttitudeTarget.IGNORE_ROLL_RATE +  AttitudeTarget.IGNORE_PITCH_RATE +  AttitudeTarget.IGNORE_YAW_RATE +  AttitudeTarget.IGNORE_ATTITUDE

		elif(self.test_state == 1):
			at_out.thrust = self._widget.slider_throttle.value() / 100.0


			at_out.body_rate.x = self._widget.spinbox_setpoint_rate.value() * math.pi / 180.0
			at_out.body_rate.y = 0.0
			at_out.body_rate.z = 0.0

			at_out.orientation.w = 1.0
			at_out.orientation.x = 0.0
			at_out.orientation.y = 0.0
			at_out.orientation.z = 0.0

			at_out.type_mask = AttitudeTarget.IGNORE_ATTITUDE

		elif(self.test_state == 2):
			at_out.thrust = self._widget.slider_throttle.value() / 100.0

			at_out.body_rate.x = 0.0
			at_out.body_rate.y = 0.0
			at_out.body_rate.z = 0.0

			(at_out.orientation.x, at_out.orientation.y, at_out.orientation.z, at_out.orientation.w) = tf.transformations.quaternion_from_euler(self._widget.spinbox_setpoint_attitude.value() * math.pi / 180.0, 0.0, 0.0)

			at_out.type_mask = AttitudeTarget.IGNORE_ROLL_RATE +  AttitudeTarget.IGNORE_PITCH_RATE +  AttitudeTarget.IGNORE_YAW_RATE

		at_out.header.stamp = rospy.Time.now()
		at_out.header.frame_id = "fcu"

		self.pub_sp.publish(at_out)
		rospy.logdebug(at_out)



