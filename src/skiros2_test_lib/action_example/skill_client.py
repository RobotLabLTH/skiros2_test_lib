from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
import skiros2_common.tools.logger as log
import actionlib
import rospy
import Queue
from actionlib.msg import TestAction, TestGoal

class ActionSkillDescription(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("InputParam", 0, ParamTypes.Required)
        self.addParam("OutputParam", 0, ParamTypes.Optional)
        #=======PreConditions=========

class DriveDescription(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Mode", 'DIFFERENTIAL', ParamTypes.Required)
        self.addParam("Target", '', ParamTypes.Required)
        #=======PreConditions=========

class PrimitiveActionClient(PrimitiveBase):
    """
    @brief Base class for skills based on a action server.

    See test_action_skill for a practical example
    """
    def onPreempt(self):
        """
        @brief Cancel all goals
        """
        self.client.cancel_all_goals()
        return self.success("Stopped")

    def onStart(self):
        self.q = Queue.Queue(1)
        self._done = None
        self.client = self.buildClient()
        if not self.client.wait_for_server(rospy.Duration(0.1)):
            return self.startError("Action server is not available.", -1)
        self.client.send_goal(self.buildGoal(), done_cb= self._doneCb, feedback_cb = self._feedbackCb)
        return True

    def execute(self):
        if not self.q.empty():
            msg = self.q.get(False)
            if self._done != None:
                return self.onDone(self._done, msg)
            else:
                return self.onFeedback(msg)
        return self.step("")

    def _doneCb(self, state, msg):
        self.q.put(msg)
        self._done = state

    def _feedbackCb(self, msg):
        if self.q.empty():
            self.q.put(msg)

    def buildClient(self):
        """
        @brief To override. Called when starting the skill
        @return an action client (e.g. actionlib.SimpleActionClient)
        """
        pass

    def buildGoal(self):
        """
        @brief To override. Called when starting the skill
        @return an action msg initialized
        """
        pass

    def onFeedback(self, msg):
        """
        @brief To override. Called every time a new feedback msg is received.
        @return Can return self.success, self.fail or self.step
        """
        #Do something with feedback msg
        return self.step("")

    def onDone(self, state, msg):
        """
        @brief To override. Called when goal is achieved.
        @return self.success or self.fail
        """
        #Do something with result msg
        return self.success("Finished. State: {} Result: {}".format(state, msg))

class test_action_skill(PrimitiveActionClient):
    """
    @brief A skill that connects to a test action server

    Goal and feeback is just an integer
    """
    def createDescription(self):
        self.setDescription(ActionSkillDescription(), self.__class__.__name__)

    def buildClient(self):
        return actionlib.SimpleActionClient('/test_action_server', TestAction)

    def buildGoal(self):
        return TestGoal(self.params["InputParam"].value)

    def onFeedback(self, msg):
        self.params["OutputParam"].value = msg.feedback
        return self.step("{}".format(self.params["OutputParam"].value))

class drive_skill(PrimitiveActionClient):
    """
    A skill based on a action server implementation
    """
    def createDescription(self):
        self.setDescription(DriveDescription(), self.__class__.__name__)

    def buildClient(self):
        mode = self.params["Mode"].value
        if mode == 'DIFFERENTIAL':
            topic = '/diff_drive'
            actionmsg = TestAction
            self.actionGoal = TestGoal
        else:
            return None
        return actionlib.SimpleActionClient(topic, actionmsg)

    def buildGoal(self):
        target = self.params["Target"].value
        return self.actionGoal(target)

    def onFeedback(self, msg):
        #Do something with feedback msg
        return self.step("{}".format(self.params["OutputParam"].value))