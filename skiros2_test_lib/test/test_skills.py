from skiros2_skill.core.skill import SkillDescription, SkillBase, Selector, State, ParallelFf, SerialStar
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
from skiros2_std_skills.service_client_primitive import PrimitiveServiceClient
from rclpy import action, client
from skiros2_msgs.action import TestAction
from std_srvs.srv import SetBool

import skiros2_common.tools.logger as log

#################################################################################
# Description
#################################################################################

class TestPrimitive(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Force", 0.0, ParamTypes.Required)
        #=======PreConditions=========

class TestSkill(SkillDescription):
    def createDescription(self):
        #=======Params=========
        #=======PreConditions=========
        pass

#################################################################################
# Implementation
#################################################################################

class test_primitive(PrimitiveBase):
    def createDescription(self):
        self.setDescription(TestPrimitive(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("Stopped")

    def onInit(self):
        return True

    def onStart(self):
        self.index = 0
        f = self.params["Force"].value
        log.info("TestPrimitive", "Send force {}".format(f))
        return True

    def execute(self):
        self.index += 1
        if self.index > 10:
            return self.success("Done")
        return self.step("")


class test_skill_sequence(SkillBase):
    def createDescription(self):
        self.setDescription(TestSkill(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("TestPrimitive", ""),
            self.skill("TestPrimitive", "", specify={"Force": 1.0})
        )

class test_skill_parallel(SkillBase):
    def createDescription(self):
        self.setDescription(TestSkill(), self.__class__.__name__)

    def expand(self, skill):
        self.setProcessor(ParallelFf())
        skill.setProcessor(ParallelFf())
        skill(
            self.skill("TestPrimitive", "test_primitive"),
            self.skill("TestPrimitive", "test_primitive", specify={"Force": 1.0}),
            self.skill("TestPrimitive", "", specify={"Force": 2.0})
        )

class test_skill_sequence_of_parallels(SkillBase):
    def createDescription(self):
        self.setDescription(TestSkill(), self.__class__.__name__)

    def expand(self, skill):
        self.setProcessor(SerialStar())
        skill.setProcessor(SerialStar())
        skill(
            self.skill("TestSkill", "test_skill_parallel"),
            self.skill("TestSkill", "test_skill_parallel"),
            self.skill("TestSkill", "test_skill_parallel"),
        )

class test_action_server(PrimitiveActionClient):
    feedback_timeout_sec = 2

    def createDescription(self):
        self.setDescription(TestSkill(), self.__class__.__name__)

    def buildClient(self)->action.ActionClient:
        """pass
        @brief To override. Called when starting the skill
        @return an action client (e.g. actionlib.SimpleActionClient)
        """

        return action.ActionClient(
            self.node, 
            TestAction, 
            "/test_action")

    def buildGoal(self):
        """
        @brief To override. Called when starting the skillpass
        @return an action msg initialized
        """
        return TestAction.Goal(ticks=15)

    def onFeedback(self, msg: TestAction.Feedback):
        
        """
        @brief To override. Called every time a new feedback msg is received.
        @return Can return self.success, self.fail or self.step
        """
        #Do something with feedback msg
        return self.step("Progress: %d" % msg.progress)
    

class test_service_server(PrimitiveServiceClient):
    def createDescription(self):
        self.setDescription(TestSkill(), self.__class__.__name__)
    
    def buildClient(self)->client.Client:
        """
        @brief To override. Called when starting the skill
        @return an service client created by self.node.create_client(...)
        """
        return self.node.create_client(SetBool, "/set_bool")

    def buildRequest(self):
        """
        @brief To override. Called when starting the skill
        @return an action msg initialized
        """
        return SetBool.Request(data=True)
    
    def onDone(self, response: SetBool.Response):
        return self.success("Got response from service: %s" % response)