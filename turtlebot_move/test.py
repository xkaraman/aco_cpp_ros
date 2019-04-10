#!/usr/bin/env python
# license removed for brevity
import rospy
from math import *
from nav_msgs.msg import Path
import os
import time
import sys
import xlwt

def callback(data):
    
    #print("Callback")
    sum = 0
    for i in range(len(data.poses)-1):
        
        sum += sqrt(pow((data.poses[i+1].pose.position.x - data.poses[i].pose.position.x),2) + pow((data.poses[i+1].pose.position.y - data.poses[i].pose.position.y), 2))
	
    print(sum)
    # Initialize a workbook 
    book = xlwt.Workbook(encoding="utf-8")

    # Add a sheet to the workbook 
    sheet1 = book.add_sheet("Python Sheet 1") 

    # Write to the sheet of the workbook 
    sheet1.write(0,i, sum)
   
    

    # Save the workbook 
    book.save("spreadsheet.xls")
    
    print "wait for 15 sec..."
    time.sleep(15)
    print "Restarting..."
    #os.execv('/home/owner/catkin_ws/src/turtlebot_move/test.py',  [''])
    os.execv(sys.executable, [sys.executable] +  ['test.py'])
    rospy.spin()
def save():	
	print("save")
	
    

def subscribe():

    sub = rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, callback)   #GlobalPlanner/plan

    rospy.init_node('CalculateCosts', anonymous=True)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
   
    save()
    subscribe()

