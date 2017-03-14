#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Authors: Saurabh Bipin Chandra
#          Divya Aggarwal
#
# The following need to be done/run before this code:
# 1. roscore
# 2. roslaunch turtlebot_bringup minimal.launch
# 3. roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/hri_project/maps/my_map4.yaml
# (You may need to change the map file above.)
# 4. Change the debug variable STATIC_DEBUG_ENABLED

import time
import rospy
import pyttsx
import actionlib
from move_base_msgs.msg import *
from PyQt4 import QtCore, QtGui
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion,PoseWithCovarianceStamped
#from face_recognition.msg import faces
import math
import cv2
import cv2.cv as cv

STATIC_DEBUG_ENABLED = False

#Change to chair positions
table_position = dict()
table_covariance = dict()

# mymap3
#table_position[1] = (-6.215, -1.463, 0.0, 0.000, 0.000, 0.000, 1.000)
#start_position    = (0.0804803371429, -0.39985704422, -0.0, 0.000, 0.000, -0.886831887211, 0.462092202731)

#map_new
#start_position    = (-0.0264121294022, 0.00267863273621, 0.0, 0.000, 0.000, 0.00690895334364, 0.999976132897)

#my_map4
#start_position    = [-0.0598124265671, 0.180819630623, 0.0, 0.000, 0.000, -0.0510998924315, 0.998693547087]
#table_position[1] = [3.12, -2.12, 0.0, 0.000, 0.000, 0.000, 1.000]

#cse_3rd
start_position       = [0.0845686197281, 0.0364561080933, 0.0, 0.000, 0.000, -0.0122166301567, 0.999925374189]
table_position[0]    = [0.0845686197281, 0.0364561080933, 0.0, 0.000, 0.000, -0.0122166301567, 0.999925374189]

table_position[3202] = [3.70228947056, 0.0306402594524, 0.0, 0.000, 0.000, 0.582214755753, 0.813035041178]
table_covariance[3202] = [0.006669135234279366, -0.0010135736841311027, 0.0, 0.0, 0.0, 0.0, -0.0010135736841311027, 0.0010314105031045402, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0012669870498752149]

table_position[3204] = [6.35258489409, -0.309624591335, 0.0, 0.000, 0.000, 0.567464851222, 0.823397621218]
table_covariance[3204] = [0.009614408609870395, -0.000908498876250885, 0.0, 0.0, 0.0, 0.0, -0.000908498876250885, 0.000956289039687655, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001098129231817914]

table_position[3208] = [10.8469794762, -0.546264474035, 0.0, 0.000, 0.000, 0.574873148577, 0.818242545365]
table_covariance[3208] = [0.008932182322453741, -0.00044274033790170364, 0.0, 0.0, 0.0, 0.0, -0.00044274033790170364, 0.000976655771211421, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000906640211636339]

table_position[3219] = [22.5201716969, -1.18497509537, 0.0, 0.000, 0.000, -0.63930002795, 0.768957394309]
table_covariance[3219] = [0.014359111746102826, -0.001675579440455266, 0.0, 0.0, 0.0, 0.0, -0.001675579440455266, 0.0010177222069234215, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001289414324140933]

table_position[3203] = [-0.906258865544, -4.70418834342, 0.0, 0.000, 0.000, -0.992212013592, 0.124560507717]
table_covariance[3203] = [0.0019433048337310632, 0.0012169281260225873, 0.0, 0.0, 0.0, 0.0, 0.0012169281260225873, 0.006137101553633784, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0014894584035412865]


AT_SOURCE      = 0
AT_DESTINATION = 1

try:
	_fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
	def _fromUtf8(s):
		return s

try:
	_encoding = QtGui.QApplication.UnicodeUTF8
	def _translate(context, text, disambig):
		return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
	def _translate(context, text, disambig):
		return QtGui.QApplication.translate(context, text, disambig)


class Delivery_Bot(object):

	def __init__(self):
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.goal = MoveBaseGoal()
		self.initialize_params()
		self.speech_engine = pyttsx.init()
		voices = self.speech_engine.getProperty('voices')
		self.speech_engine.setProperty('rate', 140)
		self.speech_engine.setProperty('voice', voices[9].id)


	def initialize_params(self):
		self.current_position = AT_SOURCE
		self.table_no = 3202
		self.current_table_position = table_position[self.table_no]
		self.parcel_on_bot = 0
		self.message       = None
		self.sender_name   = None
		self.receiver_name = None
		self.temp_message  = None
		self.prev_message  = None
		self.error_count   = 0

	def _sleep(self, seconds):
		if STATIC_DEBUG_ENABLED is False:
			time.sleep(seconds)


	def update_position_and_orientation(self, element, position):
		element.pose.position.x=float(position[0])
		element.pose.position.y=float(position[1])
		element.pose.position.z=float(position[2])
		element.pose.orientation.x = float(position[3])
		element.pose.orientation.y= float(position[4])
		element.pose.orientation.z= float(position[5])
		element.pose.orientation.w= float(position[6])


	def update_covariance(self, element): 
		element.covariance[0]  = 0.25
		element.covariance[7]  = 0.25
		element.covariance[35] = 0.06853891945200942


	def update_header(self, element): 
		element.frame_id= 'map'
		element.stamp = rospy.Time.now()


	def publish_initial_pose(self):
		self.start_pos = PoseWithCovarianceStamped()
		self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10) #, latch = True

		self.update_position_and_orientation(self.start_pos.pose, start_position)
		self.update_covariance(self.start_pos.pose)
		self.update_header(self.start_pos.header)
		self._sleep(2)
		self.pub.publish(self.start_pos)
		self._sleep(2)
		self.pub.publish(self.start_pos)
		self._sleep(2)
		self.pub.publish(self.start_pos)


	def show_source_ui(self):
		self.dest_form.close()
		self.textbox_s.setText(_translate("Form", "", None))
		self.textbox_r.setText(_translate("Form", "", None))
		self.textbox_m.setText(_translate("Form", "", None))
		self.checkbox.setChecked(False)
		self.spinBox.setValue(0)
		self.initialize_params()
		print "In show_source_ui: self.parcel_on_bot = ", self.parcel_on_bot
		self.source_form.show()


	def show_repond_ui(self):
		self.dest_form.close()
		if self.receiver_name is not None:
			self.textbox_s.setText(_translate("Form", self.receiver_name, None))
		else:
			self.textbox_s.setText(_translate("Form", "", None))

		if self.sender_name is not None:
			self.textbox_r.setText(_translate("Form", self.sender_name, None))
		else:
			self.textbox_r.setText(_translate("Form", "", None))

		self.textbox_m.setText(_translate("Form", "", None))
		self.checkbox.setEnabled(False)
		self.checkbox.setChecked(False)
		self.spinBox.setValue(0)
#		self.initialize_params()
		print "In respond_source_ui: self.parcel_on_bot = ", self.parcel_on_bot
		self.source_form.show()


	def show_destination_ui(self):
		self.source_form.close()
		self.checkbox_d.setChecked(False)
		self.dest_form.show()


	def create_font(self, font_size, bold, weight):
		font = QtGui.QFont()
		font.setPointSize(font_size)
		font.setBold(bold)
		font.setWeight(weight)
		return font


	def initialize_ui(self):
		# Create default font
		self.default_font_size = 12
		self.large_font_size   = 18
		self.font_text  = self.create_font(self.default_font_size, True, 75)
		self.font_large = self.create_font(self.large_font_size, True, 75)

		self.box_width        = 220
		self.box_height       = 40
		self.label_box_width  = self.box_width
		self.label_box_height = 25
		self.start_y          = 20
		self.related_gap_y    = 10
		self.unrelated_gap_y  = 20
		self.gap_x            = 30
		self.left_box_x  = 20
		self.right_box_x = self.left_box_x + max(self.box_width, self.label_box_width) + self.gap_x
		self.msg_box_width  = 300
		self.msg_box_height = 2*(self.box_height + self.label_box_height + self.related_gap_y) + self.unrelated_gap_y
		self.window_width   = 600
		self.window_height  = 500
		self.window_start_x = 400
		self.window_start_y = 100


	def set_ui_params(self, ui_unit, start_x, start_y, length_x, length_y, name, text, font):
		ui_unit.setGeometry(QtCore.QRect(start_x, start_y, length_x, length_y))
		ui_unit.setObjectName(_fromUtf8(name))
		ui_unit.setFont(font)
		if text is not None:
			ui_unit.setText(_translate("Form", text, None))


	def create_push_button_ui(self, Form, push_button_name, start_x, start_y, text, length_x=0, length_y=0, font=None):
		push_button = QtGui.QPushButton(Form)
		if length_x == 0:
			length_x = self.box_width
		if length_y == 0:
			length_y = self.box_height
		if font == None:
			font = self.font_text
		self.set_ui_params(push_button, start_x, start_y, length_x, length_y, push_button_name, text, font)		
		return push_button


	def create_check_box_ui(self, Form, check_box_name, start_x, start_y, text, length_x=0, length_y=0, font=None):
		checkbox = QtGui.QCheckBox(Form)
		if length_x == 0:
			length_x = self.box_width
		if length_y == 0:
			length_y = self.box_height
		if font == None:
			font = self.font_large
		self.set_ui_params(checkbox, start_x, start_y, length_x, length_y, check_box_name, text, font)		
		return checkbox


	def create_spin_box_ui(self, Form, spin_box_name, start_x, start_y, max=9999, length_x=0, length_y=0, font=None):
		spinBox = QtGui.QSpinBox(Form)
		if length_x == 0:
			length_x = self.box_width
		if length_y == 0:
			length_y = self.box_height
		if font == None:
			font = self.font_large
		self.set_ui_params(spinBox, start_x, start_y, length_x, length_y, spin_box_name, None, font)			
		spinBox.setMaximum(max)
		spinBox.setValue(0)
		return spinBox


	def create_label_ui(self, Form, label_name, start_x, start_y, text, length_x=0, length_y=0, font=None):
		label = QtGui.QLabel(Form)
		if length_x == 0:
			length_x = self.label_box_width
		if length_y == 0:
			length_y = self.label_box_height
		if font == None:
			font = self.font_text
		self.set_ui_params(label, start_x, start_y, length_x, length_y, label_name, text, font)		
		return label


	def create_text_box_ui(self, Form, text_box_name, start_x, start_y, length_x=0, length_y=0, font=None):
		#text_box = QtGui.QLineEdit(Form)
		text_box = QtGui.QTextEdit(Form)		
		if length_x == 0:
			length_x = self.box_width
		if length_y == 0:
			length_y = self.box_height
		if font == None:
			font = self.font_large
		self.set_ui_params(text_box, start_x, start_y, length_x, length_y, text_box_name, None, font)		
		return text_box


	def setup_source_ui(self):
		self.source_form = QtGui.QWidget()
		Form = self.source_form

		# Complete Window
		Form.setObjectName(_fromUtf8("Form"))
		Form.move(self.window_start_x, self.window_start_y)
		Form.resize(self.window_width, self.window_height)
		Form.setWindowTitle(_translate("Form", "Delivery Robot", None))

		# ------------- LEFT SIDE OF GUI -------------
		# ############### Battery level ###############
		# label
		running_y = self.start_y
		self.create_label_ui(Form, "label_battery", self.left_box_x, running_y, "Battery Level")
		
		# Output progress bar
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.progressBar_s = QtGui.QProgressBar(Form)
		self.progressBar_s.setGeometry(QtCore.QRect(self.left_box_x, running_y, self.label_box_width, self.label_box_height))
		self.progressBar_s.setProperty("value", 0)
		self.progressBar_s.setObjectName(_fromUtf8("progressBar_s"))

		# ############### Room/Chair No ###############
		# label
		running_y = running_y + self.label_box_height + self.unrelated_gap_y
		self.create_label_ui(Form, "label", self.left_box_x, running_y, "Room No")
		
		# Input text box
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.spinBox = self.create_spin_box_ui(Form, "spinBox", self.left_box_x, running_y, max=4299)

		# ############### Sender's Name ###############
		# label
		running_y = running_y + self.box_height + self.unrelated_gap_y
		self.label_s = self.create_label_ui(Form, "label_s", self.left_box_x, running_y, "Sender's Name")
		
		# Input text box
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.textbox_s = self.create_text_box_ui(Form, "textbox_s", self.left_box_x, running_y)
		
		# ############### Receiver's Name ###############
		# label
		running_y = running_y + self.box_height + self.unrelated_gap_y
		self.label_r = self.create_label_ui(Form, "label_r", self.left_box_x, running_y, "Receiver's Name")

		# Input text box
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.textbox_r = self.create_text_box_ui(Form, "textbox_r", self.left_box_x, running_y)

		# ############### Parcel Placed Check box ###############
		running_y = running_y + self.box_height + self.unrelated_gap_y
		self.checkbox = self.create_check_box_ui(Form, "checkbox", self.left_box_x, running_y, "Parcel Placed")

		# ############### Label to denote error ###############
		running_y = running_y + self.box_height + self.unrelated_gap_y
		self.label_error = self.create_label_ui(Form, "label_error", self.left_box_x, running_y, "", 
			self.window_width - self.left_box_x, self.label_box_height, self.font_large)
		self.label_error.setStyleSheet('color: red')

		# ------------- RIGHT SIDE OF GUI -------------
		# ############### Bot Status ###############
		running_y = self.start_y
		self.label_status = self.create_label_ui(Form, "label_status", self.right_box_x, running_y, "Bot Status: Active")

		# ############### Message ###############
		# label
		running_y = running_y + self.label_box_height + self.related_gap_y
		running_y = running_y + self.label_box_height + self.unrelated_gap_y
		self.label_m = self.create_label_ui(Form, "label_m", self.right_box_x, running_y, "Message")

		# Input text box
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.textbox_m = self.create_text_box_ui(Form, "textbox_m", self.right_box_x, running_y, 
			self.msg_box_width, self.msg_box_height)

		# Message related buttons
		running_y = running_y + self.msg_box_height + self.unrelated_gap_y
		push_button_gap = 5
		push_button_len = (self.msg_box_width - 4*push_button_gap)/2
		
		self.pushButton_sub_msg = self.create_push_button_ui(Form, "pushButton_sub_msg", 
			self.right_box_x + push_button_gap, running_y, "Submit Message", length_x=push_button_len)
		
		self.pushButton_res_msg = self.create_push_button_ui(Form, "pushButton_res_msg", 
			self.right_box_x + push_button_len + 2*push_button_gap, running_y, "Reset Message", length_x=push_button_len)

		# ############### Go Button ###############
		running_y = running_y + self.box_height + self.unrelated_gap_y
		self.pushButton_go = self.create_push_button_ui(Form, "pushButton", 
			self.right_box_x + push_button_gap, running_y, "Go", length_x=2*push_button_len + push_button_gap, 
			font=self.font_large)

		#self.update_values()
		self.connect_source_ui_callbacks()


	def connect_source_ui_callbacks(self):
		QtCore.QObject.connect(self.spinBox, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.set_table_number)
		QtCore.QObject.connect(self.textbox_s, QtCore.SIGNAL(_fromUtf8("textChanged()")), self.set_sender)
		QtCore.QObject.connect(self.textbox_r, QtCore.SIGNAL(_fromUtf8("textChanged()")), self.set_receiver)
		QtCore.QObject.connect(self.textbox_m, QtCore.SIGNAL(_fromUtf8("textChanged()")), self.save_temp_message)
		QtCore.QObject.connect(self.pushButton_go, QtCore.SIGNAL(_fromUtf8("clicked()")), self.go_to_destination)
		QtCore.QObject.connect(self.pushButton_res_msg, QtCore.SIGNAL(_fromUtf8("clicked()")), self.reset_message)
		QtCore.QObject.connect(self.pushButton_sub_msg, QtCore.SIGNAL(_fromUtf8("clicked()")), self.save_message)
		QtCore.QObject.connect(self.checkbox, QtCore.SIGNAL(_fromUtf8("stateChanged(int)")), self.parcel_placed)
		QtCore.QMetaObject.connectSlotsByName(self.source_form)


	def set_table_number(self):
		self.table_no = self.spinBox.value()
		if self.table_no in table_position.keys():
			self.current_table_position = table_position[self.table_no]
		else:
			print "ERROR: Wrong Room Number"
		print self.current_table_position


	def set_sender(self):
		#self.sender_name = self.textbox_s.text()
		self.sender_name = self.textbox_s.toPlainText()		
		print "Sender is :", self.sender_name


	def set_receiver(self):
		self.receiver_name = self.textbox_r.toPlainText()
		print "Receiver is :", self.receiver_name


	def parcel_placed(self):
		self.parcel_on_bot = 1 - self.parcel_on_bot
		if self.parcel_on_bot:
			self.label_error.setText(_translate("Form", "", None))


	def save_temp_message(self):
		self.temp_message = self.textbox_m.toPlainText()
		print "Temp message is :", self.temp_message


	def save_message(self):
		self.message = self.temp_message
		print "Message Recieved: ", self.message
		self.label_error.setText(_translate("Form", "", None))


	def reset_message(self):
		self.message = None
		self.temp_message = None
		self.textbox_m.clear()
		print "Message Resetted!!! ", self.message

	def respond(self):
		#self.temp_message = self.textbox_m.toPlainText()
		print "In respond::::"
		self.show_repond_ui()


	def navigate(self):
		if STATIC_DEBUG_ENABLED == False:
			self.client.send_goal(self.goal)
			# Allow TurtleBot up to 60 seconds to complete task
			success = self.client.wait_for_result(rospy.Duration(60)) 
			state   = self.client.get_state()
		else:
			time.sleep(3)
			success = True
			state   = GoalStatus.SUCCEEDED

		if success and state == GoalStatus.SUCCEEDED:
			return True
		else:
			self.client.cancel_goal()
			return False


	def go_to_destination(self):
		if self.current_position != AT_SOURCE:
			print "ERROR: Expected current position is Source"
			return

		if self.parcel_on_bot or self.message:
			self.update_position_and_orientation(self.goal.target_pose, self.current_table_position)
			print "Go to Destination"
		else:
			self.label_error.setText(_translate("Form", "Need either a parcel or a message to deliver", None))
			self.speech_engine.say("You missed to place your parcel or message")
			self.speech_engine.runAndWait()
			self.error_count = self.error_count + 1
			return

		self.update_header(self.goal.target_pose.header)
		result = self.navigate()

		if result is True:
			print "DESTINATION REACHED"
			self.current_position = AT_DESTINATION

#			self.aligning_to_face()
			self.aligning_to_face2()
			self.generate_delivery_message()
			self.show_destination_ui()
		else:
			print "ERROR: Navigation Failed"
			self.go_to_source()


	def detect_faces(self, img, cascade):
		rects = cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=3, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
		if len(rects) == 0:
			return None
		rects[:,2:] += rects[:,:2]
		return rects

	def aligning_to_face2(self):
		current_hack = 1
		if STATIC_DEBUG_ENABLED == True and current_hack == 1:
			self.update_position_and_orientation(self.goal.target_pose, start_position)
			self.update_header(self.goal.target_pose.header)
			self.client.send_goal(self.goal)
			success = self.client.wait_for_result(rospy.Duration(60)) 
			state   = self.client.get_state()
			if success and state == GoalStatus.SUCCEEDED:
				result = True
			else:
				self.client.cancel_goal()
				result = False
			print "Moved a bit initially."

		self.aligned_to_face = False
		if STATIC_DEBUG_ENABLED == False:
			self.rotate = self.current_table_position
		else:
			self.rotate = start_position
		self.angle_in_radians = 2*math.acos(self.rotate[6])

		vc = cv2.VideoCapture(0)
		cascade = cv2.CascadeClassifier("../turtlebot_ds/src/face_recognition/data/haarcascade_frontalface_alt.xml")
		if vc.isOpened(): # try to get the first frame
			rval, frame = vc.read()
		else:
			rval = False

		while rval:
			rval, frame = vc.read()
			print "Got a frame"
			gray = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
			gray = cv2.equalizeHist(gray)
	#		gray_small = cv2.resize(gray, size(gray), 0.5, 0.5)
			face_rects  = self.detect_faces(gray, cascade)
			if face_rects is not None or current_hack == 0:
				print ">>>>>>>>>>>>>>>>>>> Detected face"
				self.aligned_to_face = True
				break
			self.rotate_bot()

	def rotate_bot(self):
		z = math.sin(self.angle_in_radians/2)
		w = math.cos(self.angle_in_radians/2)
		print "z = ", z, "  w = ", w
		self.rotate[5] = z
		self.rotate[6] = w
		self.update_position_and_orientation(self.goal.target_pose, self.rotate)
		self.update_header(self.goal.target_pose.header)
		self.client.send_goal(self.goal)
		success = self.client.wait_for_result(rospy.Duration(60)) 
		state   = self.client.get_state()
		if success and state == GoalStatus.SUCCEEDED:
			result = True
		else:
			self.client.cancel_goal()
			result = False
		self.angle_in_radians = self.angle_in_radians + 0.05
		#self.angle_in_radians = self.angle_in_radians #+ 0.05
		print "Rotated by ", -z, " ,result is ", result


	def aligning_to_face(self):
		if STATIC_DEBUG_ENABLED == False:
			self.rotate = self.current_table_position
		else:
			self.rotate = start_position
		self.z = self.rotate[5]
		self.angle_in_radians = 2*math.acos(self.rotate[6])
		print "Angles = ", 2*math.asin(self.z), self.angle_in_radians
		print "self.z = ", self.z, " self.w = ", self.rotate[6]
		#self.angle_in_radians = 0 #2*math.acos(self.rotate[6])
		self.aligned_to_face = False
		print "Going to subscribe to /face_points"
		self.face_detection_node = rospy.Subscriber("/face_points", faces, self.aligning_to_face_cb)


	def aligning_to_face_cb(self, face_rect):
		if self.aligned_to_face == True:
			return
		print "in callback for faces: ", face_rect.x1, face_rect.y1, face_rect.x2, face_rect.y2
		center_x = (face_rect.x1 + face_rect.x2)/2
#		if center_x <= 360 and center_x >= 280:
		if center_x > 0:
			print "DO NOTHING!"
			self.face_detection_node.unregister()
			self.aligned_to_face = True
		else:
			self.z = math.sin(self.angle_in_radians/2)
			w = math.cos(self.angle_in_radians/2)
			#self.z = self.z + 0.02
			#if self.z > 1:
			#	self.z = self.z - 2
			#elif self.z < -1:
			#	self.z = self.z + 2
			#w = math.sqrt(1 - self.z*self.z)
			print "self.z = ", self.z, "  w = ", w
			self.rotate[5] = -self.z
			self.rotate[6] = w
			self.update_position_and_orientation(self.goal.target_pose, self.rotate)
			self.update_header(self.goal.target_pose.header)
		#		result = self.navigate()
			self.client.send_goal(self.goal)
			# Allow TurtleBot up to 60 seconds to complete task
			success = self.client.wait_for_result(rospy.Duration(60)) 
			state   = self.client.get_state()
			if success and state == GoalStatus.SUCCEEDED:
				result = True
			else:
				self.client.cancel_goal()
				result = False
			self.angle_in_radians = self.angle_in_radians + 0.17
			print "Rotated by ", self.z, " ,result is ", result


	def generate_delivery_message(self):
		delivery_msg = "Hi "
		if self.receiver_name:
			delivery_msg = delivery_msg + self.receiver_name + " ,\n"

		if self.parcel_on_bot and self.message:
			delivery_msg = delivery_msg + "You have an item\nand a message "
		elif self.parcel_on_bot:
			delivery_msg = delivery_msg + "You have an item "
		elif self.message:
			delivery_msg = delivery_msg + "You have a message "
		if self.sender_name:
			delivery_msg = delivery_msg + "\nfrom " + self.sender_name

		print delivery_msg
		self.speech_engine.say(delivery_msg)
		self.speech_engine.runAndWait()
		self.label_dm.setText(_translate("Form", delivery_msg, None))


	def setup_destination_ui(self):
		self.dest_form = QtGui.QWidget()
		Form = self.dest_form

		# Complete Window
		Form.setObjectName(_fromUtf8("Form_d"))
		Form.move(self.window_start_x, self.window_start_y)
		Form.resize(self.window_width, self.window_height)
		Form.setWindowTitle(_translate("Form_d", "Delivery Robot", None))

		# ------------- RIGHT SIDE OF GUI -------------
		# ############### Bot Status ###############
		running_y = self.start_y
		self.label_status_d = self.create_label_ui(Form, "label_status_d", self.right_box_x, running_y, "Bot Status: Active")

		# ############### Message ###############
		# Message related buttons
		running_y = running_y + self.label_box_height + self.related_gap_y
		running_y = running_y + self.label_box_height + self.unrelated_gap_y
		push_button_gap = 5
		push_button_len = (self.msg_box_width - 4*push_button_gap)/2
		
		self.pushButton_show_msg = self.create_push_button_ui(Form, "pushButton_show_msg", 
			self.right_box_x + push_button_gap, running_y, "Show Message", length_x=push_button_len)
		
		self.pushButton_clear_msg = self.create_push_button_ui(Form, "pushButton_clear_msg", 
			self.right_box_x + push_button_len + 2*push_button_gap, running_y, "Clear Message", length_x=push_button_len)

		# label
		running_y = running_y + self.box_height + self.unrelated_gap_y
		right_message_label_y = running_y
		self.create_label_ui(Form, "label_m", self.right_box_x, running_y, "Message")

		# Input text box
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.textbox_m_d = self.create_text_box_ui(Form, "textbox_m_d", self.right_box_x, running_y, 
			self.msg_box_width, self.msg_box_height)

		# ############### Respond Button ###############
		running_y = running_y + self.msg_box_height + self.unrelated_gap_y
		right_go_button_y = running_y
		self.pushButton_respond = self.create_push_button_ui(Form, "pushButton_respond", 
			self.right_box_x + push_button_gap, running_y, "Respond", length_x=2*push_button_len + push_button_gap, 
			font=self.font_large)
		self.pushButton_respond.setEnabled(False)

		# ############### Go Button ###############
		running_y = running_y + self.box_height + self.unrelated_gap_y
		self.pushButton_go_d = self.create_push_button_ui(Form, "pushButton_go_d", 
			self.right_box_x + push_button_gap, running_y, "Go Home", length_x=2*push_button_len + push_button_gap, 
			font=self.font_large)


		# ------------- LEFT SIDE OF GUI -------------
		# ############### Battery level ###############
		# label
		running_y = self.start_y
		self.create_label_ui(Form, "label_battery", self.left_box_x, running_y, "Battery Level")
		
		# Output progress bar
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.progressBar_d = QtGui.QProgressBar(Form)
		self.progressBar_d.setGeometry(QtCore.QRect(self.left_box_x, running_y, self.label_box_width, self.label_box_height))
		self.progressBar_d.setProperty("value", 0)
		self.progressBar_d.setObjectName(_fromUtf8("progressBar_d"))

		# ############### Destination Main Message ###############
		# label
		running_y = right_message_label_y
		self.label_dm = self.create_label_ui(Form, "label_dm", self.left_box_x, running_y, "", 
			self.label_box_width, 4*self.box_height, self.font_large)

		# ############### Parcel Received Check box ###############
		running_y = right_go_button_y + 5
		self.checkbox_d = self.create_check_box_ui(Form, "checkbox_d", self.left_box_x, running_y, 
			"Parcel Received", self.label_box_width, self.label_box_height)

		# ############### Label to denote error ###############
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.label_error_d = self.create_label_ui(Form, "label_error_d", self.left_box_x, running_y, "", 
			self.label_box_width, 2*self.label_box_height+5, self.font_large)
		self.label_error_d.setStyleSheet('color: red')

		#self.update_values()
		self.connect_destination_ui_callbacks()


	def connect_destination_ui_callbacks(self):
		QtCore.QObject.connect(self.pushButton_show_msg, QtCore.SIGNAL(_fromUtf8("clicked()")), self.show_message)
		QtCore.QObject.connect(self.pushButton_clear_msg, QtCore.SIGNAL(_fromUtf8("clicked()")), self.clear_message)
		QtCore.QObject.connect(self.checkbox_d, QtCore.SIGNAL(_fromUtf8("stateChanged(int)")), self.parcel_received)
		QtCore.QObject.connect(self.pushButton_go_d, QtCore.SIGNAL(_fromUtf8("clicked()")), self.go_to_source)
		QtCore.QObject.connect(self.pushButton_respond, QtCore.SIGNAL(_fromUtf8("clicked()")), self.respond)
		QtCore.QMetaObject.connectSlotsByName(self.dest_form)


	def show_message(self):
		if self.message:
			self.textbox_m_d.setText(_translate("Form", self.message, None))


	def clear_message(self):
		self.textbox_m_d.setText(_translate("Form", "", None))
		self.prev_message = self.message
		self.message = None
		self.temp_message = None


	def parcel_received(self):
		self.parcel_on_bot = 0
		self.label_error_d.setText(_translate("Form", "", None))


	def go_to_source(self):
		if self.current_position != AT_DESTINATION:
			print "ERROR: Expected current position is Destination"
			return

		if self.parcel_on_bot == 0:
			self.update_position_and_orientation(self.goal.target_pose, start_position)
			print "Go Home"
			self.clear_message()
		else:
			self.label_error_d.setText(_translate("Form", "Please pick up your \nparcel", None))
			self.speech_engine.say("Don't forget your parcel")
			self.speech_engine.runAndWait()
			self.error_count = self.error_count + 1
			return

		self.update_header(self.goal.target_pose.header)
		result = self.navigate()

		if result is True:
			print "HOME REACHED"
			print "Error Count = ", self.error_count
			self.current_position = AT_SOURCE
			self.show_source_ui()
		else:
			print "ERROR: Navigation Failed"
			self.go_to_source()




	#def update_values(self):
	  	#self.thread =  WorkThread() 
	  	#QtCore.QObject.connect( self.thread,  QtCore.SIGNAL("update(QString)"), self.add )
	  	#self.thread.start()


#	def add(self,text):
#		battery_value = rospy.get_param("battery_value")
#		robot_status  = rospy.get_param("robot_status")
#		self.progressBar_s.setProperty("value", battery_value)
#		self.progressBar_d.setProperty("value", battery_value)
#		self.label_status.setText(_fromUtf8(robot_status))
#		self.label_status_d.setText(_fromUtf8(robot_status))


class WorkThread(QtCore.QThread):
	def __init__(self):
		QtCore.QThread.__init__(self)
 
	def __del__(self):
		self.wait()
 
	def run(self):
		while True:
			time.sleep(0.3) # artificial time delay
			self.emit( QtCore.SIGNAL('update(QString)'), " " ) 
			#print "WorkThread>> Updating values"
  		return



if __name__ == "__main__":
	import sys

	rospy.init_node('robot_gui')
	rospy.set_param('battery_value',0)
	rospy.set_param('robot_status'," ")

	app  = QtGui.QApplication(sys.argv)
	db   = Delivery_Bot()
	db.publish_initial_pose()
	db.initialize_ui()
	db.setup_source_ui()
	db.setup_destination_ui()
	db.show_source_ui()

	sys.exit(app.exec_())
