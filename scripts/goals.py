#!/usr/bin/env python

import rospy
from math import *
from geometry_msgs.msg import PointStamped,PoseStamped,Pose,Quaternion
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import String,Header


class WayPoints:
	def __init__(self):
		self.click_sub = rospy.Subscriber("clicked_point", PointStamped, self.callback)
		self.done_sub = rospy.Subscriber("calc_costs", String , self.calc_costs)
		self.path_pub = rospy.Publisher("calc_path", Path, queue_size = 10)
		self.waypoints = list()

	def callback(self,data):
		print("### Data Received ###")		
		print(data)
		self.waypoints.append(data)
		print("\n### Data in List ###")
		for p in self.waypoints:
			print(p)		

	def calc_costs(self,data):
		rospy.wait_for_service('/move_base/make_plan');
		print("### Calculating Path Cost ###")
		for i in range(len(self.waypoints)):
			for j in range(i,len(self.waypoints)):
				make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
   				hdr = Header(stamp=rospy.Time.now(), frame_id='map')
				start = PoseStamped(header=hdr, pose=Pose(self.waypoints[i].point,Quaternion(0,0,0,1)))				
				stop = PoseStamped(header=hdr, pose=Pose(self.waypoints[j].point,Quaternion(0,0,0,1)))		
				resp = make_plan_service(start,stop,0.2);
				
				
				path_cost = self.calc_path_cost(resp.plan.poses)
				print(len(resp.plan.poses), path_cost)
				resp.plan.header.frame_id = 'map'
				resp.plan.header.stamp = rospy.get_rostime()
				self.path_pub.publish(resp.plan)
				rospy.sleep(2)
		print("### Done ###")				

	def calc_path_cost(self,poses):
		sum = 0
		for i in range(len(poses)-1):
			sum += sqrt(pow((poses[i+1].pose.position.x - poses[i].pose.position.x),2) + pow((poses[i+1].pose.position.y - poses[i].pose.position.y), 2))
		return sum


if __name__ == '__main__':
	rospy.init_node('calc_costs', anonymous = True)
	wp = WayPoints()
	rospy.spin()
