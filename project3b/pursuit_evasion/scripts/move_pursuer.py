#!/usr/bin/env python
# Copyright 2017 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy

import math
import time
import sys

from tf import TransformListener
from std_msgs.msg import String,Float32
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, RegionOfInterest

class MultiGoals:
    def __init__(self):
        topic_1 = '/tb3_0/move_base/result'
        topic_2 = '/tb3_0/move_base_simple/goal' 
	self.distance=rospy.Subscriber("/pursuer_distance",Float32,self.callback)
	self.offset=rospy.Subscriber("/pursuer_detections",RegionOfInterest,self.offsetcallback)    
        self.sub = rospy.Subscriber(topic_1, MoveBaseActionResult,queue_size=10)
        self.pub = rospy.Publisher(topic_2, PoseStamped, queue_size=10)
        self.pl = None
	self.dist = None
        self.off = None
        self.new_goal = False
        
    def recalculate_goal(self):
        if(self.dist is not None and self.off is not None):
            pl = PoseStamped()
            pl.header.frame_id = "tb3_0/base_link"
            pl.pose.position.x = mg.dist - 0.5
            pl.pose.position.y = -mg.off
            pl.pose.orientation.w = 1.0  

            self.pl = pl
            self.new_goal = True

            rospy.loginfo("New Goal:")
            rospy.loginfo(pl)


    def offsetcallback(self,data):
        # heuristic values on distance from center and an approximate "pixels to meters" conversion
        # ideally that conversion should come from camera parameters
        self.off = (data.x_offset + data.width / 2 - 640 / 2) * 8.0/640
        self.recalculate_goal()
	#rospy.loginfo(data)


    def callback(self,msg):
	self.dist=msg.data
        self.recalculate_goal()
	#rospy.loginfo(msg)
     


if __name__ == "__main__":
    try:
        rospy.init_node('multi_goals', anonymous=True)
        rate = rospy.Rate(1)
	tf = TransformListener()
	mg=MultiGoals()
	
        start_time = rospy.get_rostime()
       	while not rospy.is_shutdown():
            if mg.new_goal:
                try:
                    pl=tf.transformPose("map", mg.pl)
                    mg.pub.publish(pl)  
                    mg.new_goal = False
                except Exception as e:
                        rospy.logerr(e)
			
            # die after 5 minutes
            if rospy.get_rostime().secs - start_time.secs > 600:
                break
            rate.sleep()		    
            

    except KeyboardInterrupt:
    	print("shutting down")
