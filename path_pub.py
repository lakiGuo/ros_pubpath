#!/usr/bin/env python

import rospy
import numpy as np
from ros_a_star import Astar
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point,Pose,PoseStamped,PoseArray,Quaternion
from geometry_msgs.msg import PolygonStamped,Polygon,Point32,PoseWithCovarianceStamped,PointStamped
from nav_msgs.msg import Path

class Path_my_Astar:
	def __init__(self):
		rospy.init_node("path_pub")
		self.path_pub=rospy.Publisher("/Path_my_Astar",Path,queue_size=15)

		self.origin_x=0
		self.origin_y=0
		self.reso=0
		self.width=0
		self.height=0
		
		self.map_sub=rospy.Subscriber("/map",OccupancyGrid,self.map_callback)
		self.current_path=Path()
		rospy.sleep(1)
		
		self.start_x=0
		self.start_y=0

		self.goal_x=0
		self.goal_y=0
		self.if_start_find_path=False
		self.path_world=[]
		self.path_map=[]
		
		self.goal_pose = PoseStamped()
		self.init_pose = PoseWithCovarianceStamped()
		self.init_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pose_callback)
		self.goal_pose_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)
		self.last_time = rospy.get_rostime()
		self.start_find_path()
		rospy.Rate(1)
		rospy.spin()

	def init_pose_callback(self,msg):
		self.init_pose=msg
		self.start_x=msg.pose.pose.position.x
		self.start_y=msg.pose.pose.position.y
		self.start=[self.start_x,self.start_y]
		print "start point---- ",self.start
	
	def goal_pose_callback(self,msg):
		self.path_world=[]	
		self.init_pose=msg
		self.if_start_find_path=True
		self.goal_x=msg.pose.position.x
		self.goal_y=msg.pose.position.y
		self.goal=[self.goal_x,self.goal_y]
		print "goal point-----",self.goal
		self.start_find_path()

	def map_callback(self,msg):
		print msg.header
		print "------"
		print msg.info
		print "------"
		print len(msg.data)
		self.origin_x = msg.info.origin.position.x
		self.origin_y = msg.info.origin.position.y
		self.resolution = msg.info.resolution
		self.width = msg.info.width
		self.height = msg.info.height
		print "-------",self.width
		print "-------",self.height
		raw = np.array(msg.data, dtype=np.int8)
		self.map1 = raw.reshape((self.height, self.width))
		self.map_sub.unregister()

	def start_find_path(self):
		if self.if_start_find_path:
			print "start find Astar!"
			self.path_map=Astar(self.map1,self.resolution,15,self.start,self.goal,self.origin_x,self.origin_y).find_path()
			print len(self.path_map),len(self.path_map[0])
			self.publisher_path()
		else:
			rospy.sleep(1)
			print "please set goal pose"
			return
	def publisher_path(self):
                time=1
                for i in range(len(self.path_map)):
			current_time=rospy.get_rostime()
			current_pose=PoseStamped()
			current_pose.pose.position.x,current_pose.pose.position.y=self.path_map[i][0],self.path_map[i][1]
			#print current_pose.pose.position.x,current_pose.pose.position.y
			current_pose.pose.position.z=0.0
			current_pose.pose.orientation.x=0.0
			current_pose.pose.orientation.y=0.0
			current_pose.pose.orientation.z=0.0
			current_pose.pose.orientation.w=1.0

			time+=1
			self.current_path.header.stamp=current_time
			self.current_path.header.frame_id="odom"
			self.current_path.poses.append(current_pose)
			self.path_pub.publish(self.current_path)
			self.last_time=current_time



if __name__=='__main__':
	start_P=Path_my_Astar()


