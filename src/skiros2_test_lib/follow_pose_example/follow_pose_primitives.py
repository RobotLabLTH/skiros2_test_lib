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

from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase

#################################################################################
# Descriptions
#################################################################################
    
class PoseGenerator(SkillDescription):
    def createDescription(self):
        self._type = ":PoseGenerator"
        #=======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Optional)
        self.addParam("Pose2", Element("skiros:TransformationPose"), ParamTypes.Optional)
        
class PoseMover(SkillDescription):
    def createDescription(self):
        self._type = ":PoseMover"
        #=======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.World)
        self.addParam("Direction", 0, ParamTypes.Config, description="x: 0, y: 1, z: 2")
        
class PoseFollower(SkillDescription):
    def createDescription(self):
        self._type = ":PoseFollower"
        #=======Params=========
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.World)
        self.addParam("Pose2", Element("skiros:TransformationPose"), ParamTypes.World)

#################################################################################
# Implementations
#################################################################################

class pose_generator(PrimitiveBase):
    """
    This primitive has 3 states
    """
    def createDescription(self):
        self.setDescription(PoseGenerator(), self.__class__.__name__)
        
    def execute(self):
        if self._progress_code==0:
            return self.step("Start")
        elif self._progress_code==1:
            return self.step("Continue")
        else:
            pose = self.params["Pose"].value
            if pose._id=="":
                pose.setData(":Position", [0.0,0.0,0.0])
                pose.setData(":Orientation", [0.0,0.0,0.0,1.0])
                pose.addRelation("skiros:Scene-0", "skiros:contain", "-1")
                self.params["Pose"].value = pose
            pose2 = self.params["Pose2"].value
            if pose2._id=="":
                pose2.setData(":Position", [-1.0,-1.0,-1.0])
                pose2.setData(":Orientation", [0.0,0.0,0.0,1.0])
                pose2.addRelation("skiros:Scene-0", "skiros:contain", "-1")
                self.params["Pose2"].value = pose2
            return self.success("Done")
        
class linear_mover(PrimitiveBase):
    """
    This primitive has 1 state when progress is < 10 and 1 state of success
    """
    def createDescription(self):
        self.setDescription(PoseMover(), self.__class__.__name__)
                
    def execute(self):
        pose = self.params["Pose"].value
        direction = self._params.getParamValue("Direction")
        position = pose.getData(":Position")
        position[direction] = position[direction] + 0.1
        pose.setData(":Position", position)  
        self.params["Pose"].value = pose
        if self._progress_code<10:
            return self.step("Changing position to: {}".format(position))
        else:
            return self.success("Done")
        
class angular_mover(PrimitiveBase):
    """
    This primitive has 1 state when progress is < 10 and 1 state of success
    """
    def createDescription(self):
        self.setDescription(PoseMover(), self.__class__.__name__)
                
    def onPreempt(self):
        return self.success("Done")
        
    def execute(self):
        pose = self.params["Pose"].value
        o = pose.getData(":OrientationEuler")
        d = self._params.getParamValue("Direction")
        o[d] = o[d] + 0.1
        pose.setData(":OrientationEuler", o)  
        self.params["Pose"].value = pose
        if self._progress_code<10:
            return self.step("Changing orientation to: {}".format(o))
        else:
            return self.success("Done")
            
#import math

class rotation_mover(PrimitiveBase):
    """
    """
    def createDescription(self):
        self.setDescription(PoseMover(), self.__class__.__name__)
                
    def onPreempt(self):
        return self.success("Done")
        
    def execute(self):
        pose = self.params["Pose"].value
#        p = pose.getData(":Position")
        o = pose.getData(":OrientationEuler")
        d = self._params.getParamValue("Direction")
        o[d] = o[d] + 0.1
        pose.setData(":OrientationEuler", o)  
        self.params["Pose"].value = pose
        return self.step("Changing orientation to: {}".format(o))
            
class pose_follower(PrimitiveBase):
    """
    This primitive doesn't stop until it is preempted explicitely
    """
    def createDescription(self):
        self.setDescription(PoseFollower(), self.__class__.__name__)
        
    def onPreempt(self):
        return self.success("Done")
        
    def execute(self):
        pose = self._params.getParamValue("Pose")
        pose2 = self._params.getParamValue("Pose2")
        position = pose.getData(":Position")
        position2 = pose2.getData(":Position")
        for i in range(0, 3):
            diff = position2[i]-position[i]
            if diff!=0:
                diff = diff/2
            position2[i] -= diff
        pose2.setData(":Position", position2)  
        self.params["Pose2"].value = pose2  
        return self.step("Following pose: {}".format(position))