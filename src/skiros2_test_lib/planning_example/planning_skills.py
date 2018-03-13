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

from skiros2_skill.core.skill import SkillDescription, SkillBase, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element


#################################################################################
# Description
#################################################################################


class Locate(SkillDescription):
    def createDescription(self):
        self._type = ":Locate"
        #=======Params=========
        self.addParam("Container", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Product"), ParamTypes.Optional)
        self.addParam("Camera", Element("skiros:Camera"), ParamTypes.Required, [ParamOptions.Lock])
        #=======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "Container", True))
        self.addPreCondition(self.getAbsRelationCond("ContainerForObject", "skiros:partReference", "Container", "Object", True));
        #=======PostConditions=========
        self.addPostCondition(self.getRelationCond("InContainer", "skiros:contain", "Container", "Object", True));
        self.addPostCondition(self.getHasPropCond("HasPosition", "skiros:Position", "Object", True))

class Drive(SkillDescription):
    def createDescription(self):
        self._type = ":Drive"
        #=======Params=========
        self.addParam("StartLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
        #=======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "StartLocation", True))
        #=======PostConditions=========
        self.addPostCondition(self.getRelationCond("NoRobotAt", "skiros:at", "Robot", "StartLocation", False))
        self.addPostCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "TargetLocation", True))

class Pick(SkillDescription):
    def createDescription(self):
        self._type = ":Pick"
        #=======Params=========
        self.addParam("Container", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Product"), ParamTypes.Optional)
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Required)
        #=======PreConditions=========
        self.addPreCondition(self.getPropCond("EmptyHanded", "skiros:ContainerState", "Gripper", "=", "Empty", True))
        self.addPreCondition(self.getRelationCond("RobotAtLocation", "skiros:at", "Robot", "Container", True))
        self.addPreCondition(self.getRelationCond("ObjectInContainer", "skiros:contain", "Container", "Object", True));
        #=======PostConditions=========
        self.addPostCondition(self.getPropCond("EmptyHanded", "skiros:ContainerState", "Gripper", "=", "Empty", False))
        self.addPostCondition(self.getRelationCond("RobotAtLocation", "skiros:at", "Robot", "Container", True))
        self.addPostCondition(self.getRelationCond("Holding", "skiros:contain", "Gripper", "Object", True))

class Place(SkillDescription):
    def createDescription(self):
        self._type = ":Place"
        #=======Params=========
        self.addParam("PlacingLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Product"), ParamTypes.Required)
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Required)
        #=======PreConditions=========

        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "PlacingLocation", True))
        self.addPreCondition(self.getRelationCond("Holding", "skiros:contain", "Gripper", "Object", True));
        #self.addPreCondition(self.getPropCond("LocationEmpty", ":ContainerState", "PlacingLocation", "=", "Empty", True));
        #=======PostConditions=========
        self.addPostCondition(self.getRelationCond("NotHolding", "skiros:contain", "Gripper", "Object", False));
        #self.addPostCondition(self.getPropCond("LocationEmpty", ":ContainerState", "PlacingLocation", "=", "Empty", False));
        self.addPostCondition(self.getPropCond("EmptyHanded", "skiros:ContainerState", "Gripper", "=", "Empty", True));
        self.addPostCondition(self.getRelationCond("InPlace", "skiros:contain", "PlacingLocation", "Object", True));

#################################################################################
# Implementation
#################################################################################


class locate_fake(SkillBase):
    """

    """
    def createDescription(self):
        self.setDescription(Locate(), self.__class__.__name__)

    def expand(self, skill):
        pass

class drive_fake(SkillBase):
    """

    """
    def createDescription(self):
        self.setDescription(Drive(), self.__class__.__name__)

    def expand(self, skill):
        pass

class pick_fake(SkillBase):
    """

    """
    def createDescription(self):
        self.setDescription(Pick(), self.__class__.__name__)

    def expand(self, skill):
        pass

class place_fake(SkillBase):
    """

    """
    def createDescription(self):
        self.setDescription(Place(), self.__class__.__name__)

    def expand(self, skill):
        pass


