#!/usr/bin/env python
import bluetooth
import decimal
import socket, string

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Imu

#Bluetooth var
name="bt_server"
target_name="siggen"
#uuid="00001100-0000-1000-8000-00805F9B34FB"
serverSocket = None
inputSocket = None
outputSocket = None

#Data
#milli = 0
accX = 0
accY = 0
accZ = 0
angVelX = 0
angVelY = 0
angVelZ = 0
orientationX = 0
orientationY = 0
orientationZ = 0
orientationW = 0
lowPassK = 0.9

#ROS var
pub = None

# TODO add calibration

def sendToROS(milli, caccX, caccY, caccZ, cangVelX, cangVelY, cangVelZ, corientationX, corientationY, corientationZ, corientationW):
	global pub
	#global milli
	global accX
	global accY
	global accZ
	global angVelX
	global angVelY
	global angVelZ
	global orientationX
	global orientationY
	global orientationZ
	global orientationW
	global lowPassK

	imuMsg = Imu()
	imuMsg.header.stamp = rospy.Time.from_sec(float(milli) / 1000)
	imuMsg.header.frame_id = "/imu"
	
	accX = accX * lowPassK + (1 - lowPassK) * caccX
	accY = accY * lowPassK + (1 - lowPassK) * caccY
	accZ = accZ * lowPassK + (1 - lowPassK) * caccZ
	angVelX = angVelX * lowPassK + (1 - lowPassK) * cangVelX
	angVelY = angVelY * lowPassK + (1 - lowPassK) * cangVelY
	angVelZ = angVelZ * lowPassK + (1 - lowPassK) * cangVelZ
	orientationX = orientationX * lowPassK + (1 - lowPassK) * corientationX
	orientationY = orientationY * lowPassK + (1 - lowPassK) * corientationY
	orientationZ = orientationZ * lowPassK + (1 - lowPassK) * corientationZ
	orientationW = orientationW * lowPassK + (1 - lowPassK) * corientationW
	
	imuMsg.linear_acceleration.x = accX
	imuMsg.linear_acceleration.y = accY
	imuMsg.linear_acceleration.z = accZ
	imuMsg.linear_acceleration_covariance[0] = 0.01
	imuMsg.linear_acceleration_covariance[4] = 0.01
	imuMsg.linear_acceleration_covariance[8] = 0.01

	imuMsg.angular_velocity.x = angVelX
	imuMsg.angular_velocity.y = angVelY
	imuMsg.angular_velocity.z = angVelZ
	imuMsg.angular_velocity_covariance[0] = 0.0025
	imuMsg.angular_velocity_covariance[4] = 0.0025
	imuMsg.angular_velocity_covariance[8] = 0.0025
	
	imuMsg.orientation.x = orientationX
	imuMsg.orientation.y = orientationY
	imuMsg.orientation.z = orientationZ
	imuMsg.orientation.w = orientationW
	imuMsg.orientation_covariance[0] = 0.001
	imuMsg.orientation_covariance[4] = 0.001
	imuMsg.orientation_covariance[8] = 0.001

	pub.publish(imuMsg)

def closeBluetooth():
	global serverSocket
	global inputSocket

	inputSocket.close()
	serverSocket.close()  

def stopServer():
	# close the bluetooth
	closeBluetooth()
	print 'server shutdown'

def connectBluetooth():
	global serverSocket
	global inputSocket
	global outputSocket

	serverSocket=bluetooth.BluetoothSocket(bluetooth.RFCOMM )
	port=3 
	serverSocket.bind(("",port))

	uuid = '00001100-0000-1000-8000-00805F9B34BF'
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

def run():
	global accX
	global accY
	global accZ
	global angVelX
	global angVelY
	global angVelZ
	global orientationX
	global orientationY
	global orientationZ
	global orientationW
	global pub

	rospy.init_node('androidIMU', anonymous=True)
	pub = rospy.Publisher('/imu', Imu)
	lowPassK = float(rospy.get_param('~low_pass_k', '0.9'))
    
	connectBluetooth()
	while not rospy.is_shutdown():
		# Receive data or close connection
		try:
			data = inputSocket.recv(1024)
			#print data
       		#Parse and send data
   		except IOError:
   			print 'error: bluetooh disconnected'
   			break
	   	milli, caccX, caccY, caccZ, cangVelX, cangVelY, cangVelZ, corientationW, corientationX, corientationY, corientationZ = data.split(" ")
	   	sendToROS(milli, caccX, caccY, caccZ, cangVelX, cangVelY, cangVelZ, corientationW, corientationX, corientationY, corientationZ)
	#Stop properly the server
	stopServer();

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass
