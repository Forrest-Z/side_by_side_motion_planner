#!/usr/bin/env python
import bluetooth
import decimal
import socket, string

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Joy

#Bluetooth var
name="bt_server"
uuid="00001100-0000-1000-8000-00805F9B34FB"
serverSocket = None
inputSocket = None
outputSocket = None

#Data
X = 1
Y = 1
driveButton = 0

#ROS var
pub = None


def initRosPy():
	global pub

	rospy.init_node('joystickApp', anonymous=True)
	pub = rospy.Publisher('joystickApp', Joy)

def sendToROS():
	global pub
	global X
	global Y
	global driveButton

	joyMsg = Joy()
	joyMsg.axes = [float(X), float(Y)]
	joyMsg.buttons = [int(driveButton)]
	joyMsg.header.stamp = rospy.Time.now()

	pub.publish(joyMsg)

def stopMovementOrder():
	joyMsg = Joy()
	joyMsg.axes = [0, 0]
	joyMsg.buttons = [0]
	joyMsg.header.stamp = rospy.Time.now()
	pub.publish(joyMsg)

def closeBluetooth():
	global serverSocket
	global inputSocket

	inputSocket.close()
	serverSocket.close()  

def stopServer():
	#Send a command to stop the wheelchair, then close the bluetooth
	stopMovementOrder()
	closeBluetooth()
	print 'server shutdown'

def connectBluetooth():
	global serverSocket
	global inputSocket
	global outputSocket

	serverSocket=bluetooth.BluetoothSocket(bluetooth.RFCOMM )
	port=3 #c'est important.
	serverSocket.bind(("",port))


	print "Listening for connections on port: ", port   
	serverSocket.listen(1)

	bluetooth.advertise_service( serverSocket, name, 
	  service_id = uuid,
	  service_classes = [ uuid, bluetooth.SERIAL_PORT_CLASS ],
	  profiles = [ bluetooth.SERIAL_PORT_PROFILE ] )
	
	port=serverSocket.getsockname()[1]
	inputSocket, address = serverSocket.accept()
	
	print "Got connection with" , address
	inputSocket.send("Connected")



def debugPrint():
	global X
	global Y
	global driveButton

	print float(X)
	print float(Y)
	print int(driveButton)

def run():
	global X
	global Y
	global driveButton

	initRosPy()
	connectBluetooth()
	while not rospy.is_shutdown():

		# Receive data or close connection
		try:
			data = inputSocket.recv(1024)
			print data
       		except IOError:
       			print 'error: bluetooh disconnected'
       			break
			
       		#Parse and send data
	       	X, Y, driveButton = data.split(" ")
		debugPrint()
	       	sendToROS()

	#Stop properly the server
	stopServer();

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass
