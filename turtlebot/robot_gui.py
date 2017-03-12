#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import *
import time
from PyQt4 import QtCore, QtGui
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion,PoseWithCovarianceStamped

#Change to chair positions
table_position = dict()
table_position[0] = (-0.465, 0.37, 0.010, 0, 0, 0.998, 0.069)
#table_position[1] = (0.599, 1.03, 0.010, 0, 0, 1.00, -0.020)
table_position[2] = (4.415, 0.645, 0.010, 0, 0, -0.034, 0.999)
table_position[3] = (7.409, 0.812, 0.010, 0, 0, -0.119, 0.993)
table_position[4] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[5] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[6] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[7] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[8] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)
table_position[9] = (1.757, 4.377, 0.010, 0, 0, -0.040, 0.999)

# mymap3
#table_position[1] = (-6.215, -1.463, 0.0, 0.000, 0.000, 0.000, 1.000)
#start_position    = (0.0804803371429, -0.39985704422, -0.0, 0.000, 0.000, -0.886831887211, 0.462092202731)

#map_new
start_position    = (-0.0264121294022, 0.00267863273621, 0.0, 0.000, 0.000, 0.00690895334364, 0.999976132897)

#my_map4
start_position    = (-0.0598124265671, 0.180819630623, 0.0, 0.000, 0.000, -0.0510998924315, 0.998693547087)
table_position[1] = (3.12, -2.12, 0.0, 0.000, 0.000, 0.000, 1.000)

STATIC_DEBUG_ENABLED = True
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
		self.current_position = AT_SOURCE
		self.table_no = 0
		self.current_table_position = table_position[self.table_no]
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.goal = MoveBaseGoal()
		self.parcel_on_bot = False
		self.message = None
		self.sender_name = None
		self.receiver_name = None
		self.temp_message = None


	def create_font(self, font_size, bold, weight):
		font = QtGui.QFont()
		font.setPointSize(font_size)
		font.setBold(bold)
		font.setWeight(weight)
		return font


	def initialize_ui(self, Form):
		self.Form = Form
		# Create default font
		self.default_font_size = 12
		self.large_font_size   = 18
		self.font_text  = self.create_font(self.default_font_size, True, 75)
		self.font_large = self.create_font(self.large_font_size, True, 75)

		self.box_width  = 200
		self.box_height = 40
		self.label_box_width  = self.box_width
		self.label_box_height = 25
		self.start_y    = 20
		self.related_gap_y    = 10
		self.unrelated_gap_y  = 20
		self.gap_x            = 30
		self.left_box_x  = 20
		self.right_box_x = self.left_box_x + max(self.box_width, self.label_box_width) + self.gap_x
		self.msg_box_width  = 300
		self.msg_box_height = 2*(self.box_height + self.label_box_height + self.related_gap_y) + self.unrelated_gap_y
		self.window_width  = 580
		self.window_height = 500


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


	def setup_source_ui(self, Form):
		# Complete Window
		Form.setObjectName(_fromUtf8("Form"))
		Form.resize(self.window_width, self.window_height)
		Form.setWindowTitle(_translate("Form", "Delivery Robot", None))

		# ------------- LEFT SIDE OF GUI -------------
		# ############### Battery level ###############
		# label
		running_y = self.start_y
		self.label_battery = self.create_label_ui(Form, "label_battery", self.left_box_x, running_y, "Battery Level")
		
		# Output progress bar
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.progressBar = QtGui.QProgressBar(Form)
		self.progressBar.setGeometry(QtCore.QRect(self.left_box_x, running_y, self.label_box_width, self.label_box_height))
		self.progressBar.setProperty("value", 0)
		self.progressBar.setObjectName(_fromUtf8("progressBar"))

		# ############### Room/Chair No ###############
		# label
		running_y = running_y + self.label_box_height + self.unrelated_gap_y
		self.label = self.create_label_ui(Form, "label", self.left_box_x, running_y, "Chair No")
		
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
		self.label_error = self.create_label_ui(self.Form, "label_error", self.left_box_x, running_y, "", 600, 25, self.font_large)
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
		self.connect_ui_callbacks(Form)


	def connect_ui_callbacks(self, Form):
		QtCore.QObject.connect(self.spinBox, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.set_table_number)
		QtCore.QObject.connect(self.textbox_s, QtCore.SIGNAL(_fromUtf8("textChanged()")), self.set_sender)
		QtCore.QObject.connect(self.textbox_r, QtCore.SIGNAL(_fromUtf8("textChanged()")), self.set_receiver)
		QtCore.QObject.connect(self.textbox_m, QtCore.SIGNAL(_fromUtf8("textChanged()")), self.save_temp_message)
		QtCore.QObject.connect(self.pushButton_go, QtCore.SIGNAL(_fromUtf8("clicked()")), self.Go)
		QtCore.QObject.connect(self.pushButton_res_msg, QtCore.SIGNAL(_fromUtf8("clicked()")), self.reset_message)
		QtCore.QObject.connect(self.pushButton_sub_msg, QtCore.SIGNAL(_fromUtf8("clicked()")), self.save_message)
		QtCore.QObject.connect(self.checkbox, QtCore.SIGNAL(_fromUtf8("stateChanged(int)")), self.parcel_placed)
		QtCore.QMetaObject.connectSlotsByName(Form)


	def parcel_placed(self):
		self.parcel_on_bot = 1 - self.parcel_on_bot
		if self.parcel_on_bot:
			self.label_error.setText(_translate("Form", "", None))


	def set_sender(self):
		#self.sender_name = self.textbox_s.text()
		self.sender_name = self.textbox_s.toPlainText()		
		print "Sender is :", self.sender_name


	def set_receiver(self):
		self.receiver_name = self.textbox_r.toPlainText()
		print "Receiver is :", self.receiver_name


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


	def publish_initial_pose(self):
		self.start_pos = PoseWithCovarianceStamped()
		self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10) #, latch = True
#		self.pub_amcl = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=10) #, latch = True

		self.start_pos.pose.pose.position.x=float(start_position[0])
		self.start_pos.pose.pose.position.y=float(start_position[1])
		self.start_pos.pose.pose.position.z=float(start_position[2])
		self.start_pos.pose.pose.orientation.x = float(start_position[3])
		self.start_pos.pose.pose.orientation.y= float(start_position[4])
		self.start_pos.pose.pose.orientation.z= float(start_position[5])
		self.start_pos.pose.pose.orientation.w= float(start_position[6])
		self.start_pos.pose.covariance[0]  = 0.25
		self.start_pos.pose.covariance[7]  = 0.25
		self.start_pos.pose.covariance[35] = 0.06853891945200942
		self.start_pos.header.frame_id= 'map'
		self.start_pos.header.stamp = rospy.Time.now()
#		time.sleep(2)
		self.pub.publish(self.start_pos)
#		time.sleep(2)
		self.pub.publish(self.start_pos)
#		time.sleep(2)
		self.pub.publish(self.start_pos)

		

	def set_table_number(self):
		self.table_no = self.spinBox.value()
		self.current_table_position = table_position[self.table_no]
		print self.current_table_position

	def Go(self):
		print "Go"
		if self.current_position == AT_SOURCE:
			if self.parcel_on_bot or self.message:
				self.goal.target_pose.pose.position.x=float(self.current_table_position[0])
				self.goal.target_pose.pose.position.y=float(self.current_table_position[1])
				self.goal.target_pose.pose.position.z=float(self.current_table_position[2])

				self.goal.target_pose.pose.orientation.x = float(self.current_table_position[3])
				self.goal.target_pose.pose.orientation.y= float(self.current_table_position[4])
				self.goal.target_pose.pose.orientation.z= float(self.current_table_position[5])
				self.goal.target_pose.pose.orientation.w= float(self.current_table_position[6])
				print "Go to destination"
			else:
				self.label_error.setText(_translate("Form", "Need either a parcel or a message to deliver", None))
				return
		else:
			self.goal.target_pose.pose.position.x=float(start_position[0])
			self.goal.target_pose.pose.position.y=float(start_position[1])
			self.goal.target_pose.pose.position.z=float(start_position[2])

			self.goal.target_pose.pose.orientation.x = float(start_position[3])
			self.goal.target_pose.pose.orientation.y= float(start_position[4])
			self.goal.target_pose.pose.orientation.z= float(start_position[5])
			self.goal.target_pose.pose.orientation.w= float(start_position[6])
			print "Go Home!! "
		self.goal.target_pose.header.frame_id= 'map'
		self.goal.target_pose.header.stamp = rospy.Time.now()

		if STATIC_DEBUG_ENABLED == False:
			self.client.send_goal(self.goal)
			# Allow TurtleBot up to 60 seconds to complete task
			success = self.client.wait_for_result(rospy.Duration(60)) 
			state   = self.client.get_state()
			result  = False
		else:
			success = True
			state = GoalStatus.SUCCEEDED
			time.sleep(3)


		if success and state == GoalStatus.SUCCEEDED:
			print "DESTINATION REACHED"
			result = True
			if self.current_position == AT_DESTINATION:
				self.current_position = AT_SOURCE
			else:
				self.current_position = AT_DESTINATION
				#self.setup_destination_ui()
		else:
			self.client.cancel_goal()


	def add(self,text):
		battery_value = rospy.get_param("battery_value")
		robot_status  = rospy.get_param("robot_status")

		self.progressBar.setProperty("value", battery_value)
		self.label_4.setText(_fromUtf8(robot_status))
 

	def setup_destination_ui(self):
		# Complete Window
		Form = self.Form
		Form.setObjectName(_fromUtf8("Form"))
		Form.resize(self.window_width, self.window_height)
		Form.setWindowTitle(_translate("Form", "Delivery Robot", None))

		# ------------- LEFT SIDE OF GUI -------------
		# ############### Battery level ###############
		# label
		running_y = self.start_y
		self.label_battery = self.create_label_ui(Form, "label_battery", self.left_box_x, running_y, "Battery Level")
		
		# Output progress bar
		running_y = running_y + self.label_box_height + self.related_gap_y
		self.progressBar = QtGui.QProgressBar(Form)
		self.progressBar.setGeometry(QtCore.QRect(self.left_box_x, running_y, self.label_box_width, self.label_box_height))
		self.progressBar.setProperty("value", 0)
		self.progressBar.setObjectName(_fromUtf8("progressBar"))

		# ############### Room/Chair No ###############
		# label
		running_y = running_y + self.label_box_height + self.unrelated_gap_y
		delivery_msg = "There is "
		if self.parcel_on_bot and self.message:
			delivery_msg = delivery_msg + "a message and an item "
		elif self.parcel_on_bot:
			delivery_msg = delivery_msg + "an item "
		elif self.message:
			delivery_msg = delivery_msg + "a message "
		delivery_msg = delivery_msg + "for " + self.receiver_name + " from " + self.sender_name

		self.label_delivery = self.create_label_ui(Form, "label", self.left_box_x, running_y, delivery_msg)
		
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
#		self.connect_ui_callbacks(Form)

	#def update_values(self):
	  	#self.thread =  WorkThread() 
	  	#QtCore.QObject.connect( self.thread,  QtCore.SIGNAL("update(QString)"), self.add )
	  	#self.thread.start()


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
	Form = QtGui.QWidget()
	db   = Delivery_Bot()
	db.publish_initial_pose()
	db.initialize_ui(Form)
	db.setup_source_ui(Form)

	Form.show()
	sys.exit(app.exec_())
