from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, Selector, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Description
#################################################################################

class TurtleFindAndFollow(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Turtle", Element("sumo:Object"), ParamTypes.Optional)

class TurtleSpawnAndFollow(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("PosX", int, ParamTypes.Required)
        self.addParam("PosY", int, ParamTypes.Required)
        self.addParam("Rotation", int, ParamTypes.Required)
        self.addParam("Name", str, ParamTypes.Required)

class TurtleSpawnAndWander(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("PosX", int, ParamTypes.Required)
        self.addParam("PosY", int, ParamTypes.Required)
        self.addParam("Rotation", int, ParamTypes.Required)
        self.addParam("Name", "turtle", ParamTypes.Required)

#################################################################################
# Implementation
#################################################################################

class patrol_and_follow(SkillBase):
    """
    Tree is:
    ----->:trajectory_coordinator (|Fs|)
    ------->:TrajectoryGenerator
    ------->:TrajectoryConsumer

    """
    def createDescription(self):
        self.setDescription(TurtleFindAndFollow(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Selector())
        skill.addChild(self.getNode(Serial()))
        skill.last().addChild(self.getSkill(":TurtleFind", ""))
        skill.last().addChild(self.getSkill(":TargetFollow", ""))
        skill.last().last().remap('Target', 'Turtle')
        skill.addChild(self.getSkill(":Wander", ""))

class stay_still_and_follow(SkillBase):
    """
    Tree is:
    ----->:trajectory_coordinator (|Fs|)
    ------->:TrajectoryGenerator
    ------->:TrajectoryConsumer

    """
    def createDescription(self):
        self.setDescription(TurtleFindAndFollow(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Selector())
        skill.addChild(self.getNode(Serial()))
        skill.last().addChild(self.getSkill(":TurtleFind", ""))
        skill.last().addChild(self.getSkill(":TargetFollow", ""))
        skill.last().last().remap('Target', 'Turtle')
        skill.addChild(self.getSkill(":Wait", ""))


class turtle_spawn_and_follow(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(TurtleSpawnAndFollow(), self.__class__.__name__)

    def expand(self, skill):
        skill.addChild(self.getNode(Sequential()))
        skill.last().addChild(self.getSkill(":TurtleSpawn", ""))
        skill.last().addChild(self.getSkill(":TurtleFind", ""))
        skill.last().addChild(self.getSkill(":TargetFollow", ""))
        skill.last().last().remap('Target', 'Turtle')


class turtle_spawn_and_wander(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(TurtleSpawnAndWander(), self.__class__.__name__)

    def expand(self, skill):
        skill.addChild(self.getNode(Sequential()))
        skill.last().addChild(self.getSkill(":TurtleSpawn", ""))
        skill.last().addChild(self.getNode(ParallelFf()))
        skill.last().last().addChild(self.getSkill(":Wander2", ""))
        skill.last().last().addChild(self.getSkill(":TurtleFind", ""))
        skill.last().last().addChild(self.getSkill(":TargetFollow", ""))
        skill.last().last().remap('Target', 'Turtle')
