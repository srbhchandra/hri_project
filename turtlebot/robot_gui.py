#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robot.ui'
#
# Created: Sat Feb 21 20:25:38 2015
#	  by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

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

class Ui_Form(object):

	def setupUi(self, Form):
		Form.setObjectName(_fromUtf8("Form"))
#		Form.resize(376, 338)
		Form.resize(800, 800)

		# Chair No text box
		self.spinBox = QtGui.QSpinBox(Form)
		self.spinBox.setGeometry(QtCore.QRect(20, 160, 161, 81))
		font = QtGui.QFont()
		font.setPointSize(35)
		font.setBold(True)
		font.setWeight(75)
		self.spinBox.setFont(font)
		self.spinBox.setMaximum(9999)
		self.spinBox.setObjectName(_fromUtf8("spinBox"))
		
		# label is "Room/Chair No" text
		self.label = QtGui.QLabel(Form)
		self.label.setGeometry(QtCore.QRect(20, 120, 130, 21))
		font = QtGui.QFont()
		font.setPointSize(12)
		font.setBold(True)
		font.setWeight(75)
		self.label.setFont(font)
		self.label.setObjectName(_fromUtf8("label"))

		# Senders Name text box
		# Create textbox
		self.textbox_s = QtGui.QLineEdit(Form)
		self.textbox_s.setGeometry(QtCore.QRect(20, 300, 161, 81))
		font = QtGui.QFont()
		font.setPointSize(15)
		font.setBold(True)
		font.setWeight(75)
		self.textbox_s.setFont(font)
		self.textbox_s.setObjectName(_fromUtf8("textbox_s"))
		
		# label:"Sender's Name" text
		self.label_s = QtGui.QLabel(Form)
		self.label_s.setGeometry(QtCore.QRect(20, 260, 130, 21))
		font = QtGui.QFont()
		font.setPointSize(12)
		font.setBold(True)
		font.setWeight(75)
		self.label_s.setFont(font)
		self.label_s.setObjectName(_fromUtf8("label_s"))

		# Receivers Name text box
		self.spinBox_r = QtGui.QSpinBox(Form)
		self.spinBox_r.setGeometry(QtCore.QRect(20, 440, 161, 81))
		font = QtGui.QFont()
		font.setPointSize(35)
		font.setBold(True)
		font.setWeight(75)
		self.spinBox_r.setFont(font)
		self.spinBox_r.setMaximum(9999)
		self.spinBox_r.setObjectName(_fromUtf8("spinBox_r"))
		
		# label:"Receiver's Name" text
		self.label_r = QtGui.QLabel(Form)
		self.label_r.setGeometry(QtCore.QRect(20, 400, 130, 21))
		font = QtGui.QFont()
		font.setPointSize(12)
		font.setBold(True)
		font.setWeight(75)
		self.label_r.setFont(font)
		self.label_r.setObjectName(_fromUtf8("label_r"))

		self.pushButton_go = QtGui.QPushButton(Form)
		self.pushButton_go.setGeometry(QtCore.QRect(220, 240, 131, 41))
		self.pushButton_go.setObjectName(_fromUtf8("pushButton"))
		self.pushButton_sub_msg = QtGui.QPushButton(Form)
		self.pushButton_sub_msg.setGeometry(QtCore.QRect(220, 140, 131, 41))
		self.pushButton_sub_msg.setObjectName(_fromUtf8("pushButton_sub_msg"))
		self.pushButton_res_msg = QtGui.QPushButton(Form)
		self.pushButton_res_msg.setGeometry(QtCore.QRect(220, 190, 131, 41))
		self.pushButton_res_msg.setObjectName(_fromUtf8("pushButton_res_msg"))
		self.progressBar = QtGui.QProgressBar(Form)
		self.progressBar.setGeometry(QtCore.QRect(20, 60, 118, 23))
		self.progressBar.setProperty("value", 0)
		self.progressBar.setObjectName(_fromUtf8("progressBar"))
		self.label_2 = QtGui.QLabel(Form)
		self.label_2.setGeometry(QtCore.QRect(20, 20, 111, 21))
		font = QtGui.QFont()
		font.setBold(True)
		font.setWeight(75)
		self.label_2.setFont(font)
		self.label_2.setObjectName(_fromUtf8("label_2"))
		self.label_3 = QtGui.QLabel(Form)
		self.label_3.setGeometry(QtCore.QRect(200, 20, 111, 21))
		font = QtGui.QFont()
		font.setBold(True)
		font.setWeight(75)
		self.label_3.setFont(font)
		self.label_3.setObjectName(_fromUtf8("label_3"))
		self.label_4 = QtGui.QLabel(Form)
		self.label_4.setGeometry(QtCore.QRect(190, 60, 131, 31))
		font = QtGui.QFont()
		font.setBold(True)
		font.setWeight(75)
		self.label_4.setFont(font)
		self.label_4.setText(_fromUtf8(""))
		self.label_4.setObjectName(_fromUtf8("label_4"))

		self.table_no = 0
		self.current_table_position = 0
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.goal = MoveBaseGoal()

		#self.update_values()

		self.retranslateUi(Form)
		QtCore.QObject.connect(self.spinBox, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.set_table_number)
		QtCore.QObject.connect(self.textbox_s, QtCore.SIGNAL(_fromUtf8("valueChanged(char)")), self.set_sender)
		QtCore.QObject.connect(self.spinBox_r, QtCore.SIGNAL(_fromUtf8("valueChanged(char)")), self.set_receiver)
		
		QtCore.QObject.connect(self.pushButton_go, QtCore.SIGNAL(_fromUtf8("clicked()")), self.Go)
		QtCore.QObject.connect(self.pushButton_res_msg, QtCore.SIGNAL(_fromUtf8("clicked()")), self.reset_message)
		QtCore.QObject.connect(self.pushButton_sub_msg, QtCore.SIGNAL(_fromUtf8("clicked()")), self.save_message)
		
		QtCore.QMetaObject.connectSlotsByName(Form)


	def retranslateUi(self, Form):
		Form.setWindowTitle(_translate("Form", "Delivery Robot", None))
		self.label.setText(_translate("Form", "Room/Chair No", None))
		self.label_s.setText(_translate("Form", "Sender's Name", None))
		self.label_r.setText(_translate("Form", "Receiver's Name", None))
		self.pushButton_go.setText(_translate("Form", "Go", None))
		self.pushButton_sub_msg.setText(_translate("Form", "Submit Message", None))
		self.pushButton_res_msg.setText(_translate("Form", "Reset Message", None))
		self.label_2.setText(_translate("Form", "Battery Level", None))
		self.label_3.setText(_translate("Form", "Robot Status", None))


	def set_sender(self):
		self.sender_name = self.textbox_s.value()
		print "Sender is :", self.sender_name


	def set_receiver(self):
		self.receiver_name = self.spinBox_r.value()
		print "Receiver is :", self.receiver_name


	def save_message(self):
		print "Message Recieved!!!"


	def reset_message(self):
		print "Message Resetted!!!"


	def set_initial_pose(self):
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
		self.current_position = AT_SOURCE
	

	def set_table_number(self):
		self.table_no = self.spinBox.value()
		self.current_table_position = table_position[self.table_no]
		print self.current_table_position

	def Go(self):
		print "Go"
		print "Waiting for server"
	#	self.client.wait_for_server()
		if self.current_position == AT_SOURCE:
			self.goal.target_pose.pose.position.x=float(self.current_table_position[0])
			self.goal.target_pose.pose.position.y=float(self.current_table_position[1])
			self.goal.target_pose.pose.position.z=float(self.current_table_position[2])

			self.goal.target_pose.pose.orientation.x = float(self.current_table_position[3])
			self.goal.target_pose.pose.orientation.y= float(self.current_table_position[4])
			self.goal.target_pose.pose.orientation.z= float(self.current_table_position[5])
			self.goal.target_pose.pose.orientation.w= float(self.current_table_position[6])
			print "Go to destination"
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

		self.client.send_goal(self.goal)

		# Allow TurtleBot up to 60 seconds to complete task
		success = self.client.wait_for_result(rospy.Duration(60)) 
		state   = self.client.get_state()
		result  = False

		if success and state == GoalStatus.SUCCEEDED:
			# We made it!
			result = True
			if self.current_position == AT_DESTINATION:
				self.current_position = AT_SOURCE
			else:
				self.current_position = AT_DESTINATION
		else:
			self.client.cancel_goal()


	def Cancel(self):
		print "Cancel"
		self.client.cancel_all_goals()


	def Home(self):
		print "Home"
		self.current_table_position = table_position[0]
		self.Go()


	def add(self,text):
		battery_value = rospy.get_param("battery_value")
		robot_status  = rospy.get_param("robot_status")

		self.progressBar.setProperty("value", battery_value)
		self.label_4.setText(_fromUtf8(robot_status))
 

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
	ui   = Ui_Form()
	ui.set_initial_pose()
	ui.setupUi(Form)
	
	Form.show()
	sys.exit(app.exec_())
