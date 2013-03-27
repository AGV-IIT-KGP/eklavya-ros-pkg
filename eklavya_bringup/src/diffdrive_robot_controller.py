#!/usr/bin/python
import roslib; 
import rospy
from serial import *
from geometry_msgs.msg import Twist


class EklavyaBotController:
	def __init__(self):
		rospy.init_node('eklavya_controller')
		rospy.loginfo("eklavya_controller Node")
		
		port_name = rospy.get_param('~port','/dev/ttyUSB0')
		baud = int(rospy.get_param('~baud','19200'))
		self.port = Serial(port_name, baud, timeout=2.5)

		self.port.timeout = 0.01
		rospy.sleep(0.1)
		rospy.loginfo("Connected on %s at %d baud" % (port_name,baud) )

		rospy.on_shutdown(self.close_ser)
		
		self.twistsub = rospy.Subscriber("cmd_vel", Twist, self.twistCallback)
		
	def twistCallback(self,msg):
		a = int(round( (msg.linear.x) + (msg.angular.z) ))
		b = int(round( (msg.linear.x) - (msg.angular.z) ))
		scale =10
		wheelSeparation=2.0

		wheelSpeed_LEFT = (a * (double)wheelSeparation / 2.0)*scale;
		wheelSpeed_RIGHT = (b * (double)wheelSeparation / 2.0)*scale;
		ba = bytearray('w')
			
		ba.append('0' + wheelSpeed_RIGHT / 10)
		ba.append('0' + wheelSpeed_RIGHT % 10)
		ba.append('0' + wheelSpeed_LEFT / 10)
		ba.append('0' + wheelSpeed_LEFT % 10)		
	
		self.port.flush()
		self.port.write(ba)

		'''el = port.readline()
		print el'''
		
	def close_ser(self):
		rospy.loginfo('Closing Port')
		self.port.close();


if __name__ == '__main__':
    try:
        eklavyaBotController = EklavyaBotController()
        rospy.spin()

    except rospy.ROSInterruptException: 
        pass
