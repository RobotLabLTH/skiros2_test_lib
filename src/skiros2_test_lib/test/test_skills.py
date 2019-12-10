from skiros2_skill.core.skill import SkillDescription, SkillBase, Selector, State, ParallelFf, SerialStar
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes

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
        print "Send force {}".format(f)
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
