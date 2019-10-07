from skiros2_skill.core.skill import SkillDescription, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import rospy
import turtlesim.msg as ts
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import threading, Queue, numpy
import math

#################################################################################
# Descriptions
#################################################################################

class Wander(SkillDescription):
    def createDescription(self):
        #=======Params=========
        pass

class Wander2(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Name", str, ParamTypes.Required)

class TurtleFind(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Turtle", Element("sumo:Object"), ParamTypes.Optional)

class TargetFollow(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        self.addParam("Pgain", 1, ParamTypes.Required)

class TurtleSpawn(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("PosX", int, ParamTypes.Required)
        self.addParam("PosY", int, ParamTypes.Required)
        self.addParam("Rotation", int, ParamTypes.Required)
        self.addParam("Name", str, ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################

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
        return True

    def execute(self):
        self._sendCmd()
        return self.step("")


class wander_around_2(PrimitiveBase):
    """
    Same as wander_around except for the name of the topic that is published (it depends on the "Name" parameters and not on the "Robot" parameter)
    """
    counter = 0

    def createDescription(self):
        self.setDescription(Wander2(), self.__class__.__name__)

    def _sendCmd(self):
        msg = Twist()
        self.counter += 0.1
        msg.linear.x = math.sin(self.counter)
        self.pose_pub.publish(msg)
        return msg

    def onReset(self):
        self.counter = 0

    def onStart(self):
        self.pose_pub = rospy.Publisher(self.params["Name"].getValue()+"/cmd_vel", Twist, queue_size=20)
        return True

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
        return True

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


class turtle_spawn(PrimitiveBase):
    """
    """
    def createDescription(self):
        self.setDescription(TurtleSpawn(), self.__class__.__name__)

    def onStart(self):
        rospy.wait_for_service('spawn')
        try:
            turtle_spawner = rospy.ServiceProxy('spawn', Spawn)
            resp = turtle_spawner(self.params["PosX"].getValue() , self.params["PosY"].getValue() , self.params["Rotation"].getValue() ,  self.params["Name"].getValue())
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return True

    def execute(self):
        return self.success("turtle spawned")
