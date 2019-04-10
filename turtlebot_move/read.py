#!/usr/bin/env python

import rospy


with open('testfile.txt', 'r') as file:
    # read a list of lines into data
    data = file.readlines()

print data
print "Your value: " + data[1]


