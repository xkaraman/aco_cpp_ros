#!/usr/bin/env python
# license removed for brevity
import rospy
from math import *
from nav_msgs.msg import Path


def callback(data):
   
    #print("Callback")
    sum = 0
    for i in range(len(data.poses)-1):
        
        sum += sqrt(pow((data.poses[i+1].pose.position.x - data.poses[i].pose.position.x),2) + pow((data.poses[i+1].pose.position.y - data.poses[i].pose.position.y), 2))
	
    print(sum)
   

	
    

def subscribe():
    
    sub = rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, callback)   #GlobalPlanner/plan

    rospy.init_node('CalculateCosts', anonymous=True)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    subscribe()
   
   

