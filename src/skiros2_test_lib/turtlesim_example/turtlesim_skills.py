from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, Selector, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Move
#################################################################################

class Wander(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)

class round_trip(SkillBase):
    def createDescription(self):
        self.setDescription(Wander(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill("Move", "move", specify={"Linear": 2.0, "Angular": 0.0}),
            self.skill("Move", "move", specify={"Linear": 0.0, "Angular": 90.0}),
            self.skill("Move", "move", specify={"Linear": 2.0, "Angular": 0.0}),
            self.skill("Move", "move", specify={"Linear": 0.0, "Angular": 90.0}),
            self.skill("Move", "move", specify={"Linear": 2.0, "Angular": 0.0}),
            self.skill("Move", "move", specify={"Linear": 0.0, "Angular": 90.0}),
            self.skill("Move", "move", specify={"Linear": 2.0, "Angular": 0.0}),
            self.skill("Move", "move", specify={"Linear": 0.0, "Angular": 90.0}),
        )

class Move(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Linear", 0.0, ParamTypes.Required)
        self.addParam("Angular", 0.0, ParamTypes.Required)
        self.addParam("Duration", 1.0, ParamTypes.Optional)

class move(SkillBase):
    def createDescription(self):
        self.setDescription(Move(), self.__class__.__name__)

    def expand(self, skill):
        velocity = self.params["Linear"].value / self.params["Duration"].value
        angular_velocity = self.params["Angular"].value / self.params["Duration"].value
        skill.setProcessor(Sequential())
        skill(
            self.skill(ParallelFs())(
                self.skill("Command", "command", specify={"Linear": velocity, "Angular": angular_velocity}),
                self.skill("Wait", "wait", specify={"Duration": self.params["Duration"].value})
            )
        )

class AttractTo(SkillDescription):
    def createDescription(self):
        self.addParam("Turtle", Element("cora:Robot"), ParamTypes.Required)
        self.addParam("Target", Element("cora:Robot"), ParamTypes.Required)

class attract_to(SkillBase):
    def createDescription(self):
        self.setDescription(AttractTo(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("Monitor", "monitor"),
            self.skill("Monitor", "monitor", remap={"Turtle": "Target"}),
            self.skill("PoseController", "pose_controller"),
            self.skill("Command", "command"),
            self.skill("Wait", "wait", specify={"Duration": 10000.0})
        )

class demo(SkillBase):
    def createDescription(self):
        self.setDescription(AttractTo(), self.__class__.__name__)

    def expand(self, skill):
        l = "Linear{}".format(self.params["Turtle"].value.label)
        a = "Angular{}".format(self.params["Turtle"].value.label)
        skill.setProcessor(ParallelFf())
        skill(
            self.skill("Wander", "round_trip", remap={"Turtle": "Target"}),
            self.skill("AttractTo", "attract_to", remap={"Linear": l, "Angular": a}),
        )
