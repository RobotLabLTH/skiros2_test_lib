#################################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Francesco Rovida
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#################################################################################

from skiros2_skill.core.skill import SkillDescription, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import rospy
import turtlesim.msg as ts
from geometry_msgs.msg import Twist
import threading, Queue, numpy
import math

#################################################################################
# Descriptions
#################################################################################
    
class Wait(SkillDescription):
    def createDescription(self):
        self._type = ":Wait"
        #=======Params=========
        
class Wander(SkillDescription):
    def createDescription(self):
        self._type = ":Wander"
        #=======Params=========
        
class TurtleFind(SkillDescription):
    def createDescription(self):
        self._type = ":TurtleFind"
        #=======Params=========
        self.addParam("Turtle", Element("sumo:Object"), ParamTypes.Optional)
        
class TargetFollow(SkillDescription):
    def createDescription(self):
        self._type = ":TargetFollow"
        #=======Params=========
        self.addParam("Target", Element("sumo:Object"), ParamTypes.World)
        self.addParam("Pgain", 2, ParamTypes.Config)

#################################################################################
# Implementations
#################################################################################
        
class wait(PrimitiveBase):
    """
    """    
    
    def createDescription(self):
        self.setDescription(Wait(), self.__class__.__name__)
        
    def onPreempt(self):
        return self.success("Done")
        
    def execute(self):
        return self.step("")
        
class wander_around(PrimitiveBase):
    """
    """    
    counter = 0
    
    def createDescription(self):
        self.setDescription(Wander(), self.__class__.__name__)
        
    def _sendCmd(self):
        msg = Twist()
        self.counter += 0.1
        msg.linear.x = math.sin(self.counter)
        self.pose_pub.publish(msg)
        return msg 
        
    def onReset(self):    
        self.counter = 0
        
    def onStart(self):    
        my_turtle = self.params["Robot"].value.getProperty("tts:TurtleName").value
        self.pose_pub = rospy.Publisher(my_turtle+"/cmd_vel", Twist, queue_size=20)
        return self.step("Start")
        
    def execute(self):
        self._sendCmd()
        return self.step("")
        
        
class turtle_find(PrimitiveBase):
    """
    """    
    def createDescription(self):
        self.setDescription(TurtleFind(), self.__class__.__name__)
        
        
    def execute(self):
        my_turtle = self.params["Robot"].value.getProperty("tts:TurtleName").value
        other_turtle = self.params["Turtle"].value 
        #Retrieve
        tlist = [topic.replace("/pose", "") for topic, type in rospy.get_published_topics() if type=='turtlesim/Pose' and topic.find(my_turtle)==-1]
        if tlist:
            other_turtle.setProperty("tts:TurtleName", tlist[0])
            self.params["Turtle"].value = other_turtle
            print self.params["Turtle"].value.printState(True)
            return self.success("Detected turtle {}".format(tlist[0]))
        #return self.step("")
        return self.fail("", -1)
        
class target_follow(PrimitiveBase):
    """
    """
    target_pose = numpy.array([0,0,0])
    self_pose = numpy.array([0,0,0])
    pose_sub = None
    pose_pub = None
    p_gain = 0
    counter = 0
    go_forward = False
    
    
    def createDescription(self):
        self.setDescription(TargetFollow(), self.__class__.__name__)
        
    def _otherPoseMonitor(self, msg):    
        self.target_pose = numpy.array([msg.x, msg.y, msg.theta])
    
    def _selfPoseMonitor(self, msg):    
        self.self_pose = numpy.array([msg.x, msg.y, msg.theta])
        
    def _sendCmd(self):
        msg = Twist()
        diff = numpy.array([(self.target_pose[0]-self.self_pose[0]), (self.target_pose[1]-self.self_pose[1])])
        tan = numpy.arctan2(*diff)-numpy.pi/2
        if tan < 0:
            tan = (2*numpy.pi + tan)
        norm = numpy.linalg.norm(diff)
        theta = abs(self.self_pose[2])
        if abs(theta-tan)>0.02 and not self.go_forward:
            msg.angular.z = self.p_gain*(theta-tan)
        else:
            self.go_forward = True
            msg.linear.x = self.p_gain*norm
        #print "{} {}".format(tan, theta)
        self.pose_pub.publish(msg)
        return msg

    def onReset(self):
        self.go_forward = False
        self.target_pose = numpy.array([0,0,0])
        self.self_pose = numpy.array([0,0,0.05])
        
    def onStart(self):    
        my_turtle = self.params["Robot"].value.getProperty("tts:TurtleName").value
        other_turtle = self.params["Target"].value.getProperty("tts:TurtleName").value
        self.opose_sub = rospy.Subscriber(other_turtle+"/pose", ts.Pose, self._otherPoseMonitor)
        self.mpose_sub = rospy.Subscriber(my_turtle+"/pose", ts.Pose, self._selfPoseMonitor)
        self.pose_pub = rospy.Publisher(my_turtle+"/cmd_vel", Twist, queue_size=20)
        return self.step("Start")
    
    def execute(self):
        self.p_gain = self.params["Pgain"].getValue()
        
        cmd = self._sendCmd()
        
        if abs(cmd.linear.x)>0.5 or abs(cmd.angular.z)>0:
            self.counter = 0
            return self.step("Speed: {} {}".format(cmd.linear.x, cmd.angular.z))
        else:
            self.counter += 1
            if self.counter>20:
                return self.success("")
        return self.step("")