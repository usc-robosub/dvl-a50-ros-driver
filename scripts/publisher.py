#!/usr/bin/env python3
import socket
import json
import rospy
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam
import select

def connect():
	global s, TCP_IP, TCP_PORT
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((TCP_IP, TCP_PORT))
		s.settimeout(1)
	except socket.error as err:
		rospy.logerr("No route to host, DVL might be booting? {}".format(err))
		sleep(1)
		connect()

oldJson = ""

theDVL = DVL()
beam0 = DVLBeam()
beam1 = DVLBeam()
beam2 = DVLBeam()
beam3 = DVLBeam()

def getData():
	global oldJson, s
	raw_data = ""

	while not '\n' in raw_data:
		try:
			rec = s.recv(1) # Add timeout for that
			if len(rec) == 0:
				rospy.logerr("Socket closed by the DVL, reopening")
				connect()
				continue
			else:
				rec = rec.decode('utf-8')
				print("rec = " + rec)
		except socket.timeout as err:
			rospy.logerr("Lost connection with the DVL, reinitiating the connection: {}".format(err))
			connect()
			continue
		raw_data = raw_data + rec
	raw_data = oldJson + raw_data
	oldJson = ""
	raw_data = raw_data.split('\n')
	oldJson = raw_data[1]
	raw_data = raw_data[0]
	return raw_data


def publisher():
	pub_raw = rospy.Publisher('dvl/json_data', String, queue_size=10)
	pub = rospy.Publisher('dvl/data', DVL, queue_size=10)
	twist_pub = rospy.Publisher('dvl/twist', TwistWithCovarianceStamped, queue_size=10)

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		raw_data = getData()
		data = json.loads(raw_data)

		# edit: the logic in the original version can't actually publish the raw data
		# we slightly change the if else statement so now
		# do_log_raw_data is true: publish the raw data to /dvl/json_data topic, fill in theDVL using velocity data and publish to dvl/data topic
		# do_log_raw_data is true: only fill in theDVL using velocity data and publish to dvl/data topic

		if do_log_raw_data:
			rospy.loginfo(raw_data)
			pub_raw.publish(raw_data)
			if data["type"] != "velocity":
				continue
		else:
			if data["type"] != "velocity":
				continue
			pub_raw.publish(raw_data)

		twist_msg = TwistWithCovarianceStamped()

		# Header
		twist_msg.header.stamp = data["time"]
		twist_msg.header.frame_id = "dvl_link"

		# Set linear velocities (m/s)
		twist_msg.twist.twist.linear.x = data["vx"]
		twist_msg.twist.twist.linear.y = data["vy"]
		twist_msg.twist.twist.linear.z = data["vz"]

		# Set angular velocities (rad/s)
		twist_msg.twist.twist.angular.x = 0.0
		twist_msg.twist.twist.angular.y = 0.0
		twist_msg.twist.twist.angular.z = 0.0

		# Set covariance (6x6 matrix as a flat array)
		covariance = data["covariance"]
		twist_msg.twist.covariance = [0.0] * 36
		dim = len(covariance)

		for i in range(dim):
			for j in range(dim):
				twist_msg.twist.covariance[i * 2 * dim + j] = covariance[i][j]

		twist_pub.publish(twist_msg)

		rate.sleep()

if __name__ == '__main__':
	global s, TCP_IP, TCP_PORT, do_log_raw_data
	rospy.init_node('a50_pub', anonymous=False)
	TCP_IP = rospy.get_param("~ip", "10.42.0.186")
	TCP_PORT = rospy.get_param("~port", 16171)
	do_log_raw_data = rospy.get_param("~do_log_raw_data", False)
	connect()
	try:
		publisher()
	except rospy.ROSInterruptException:
		s.close()