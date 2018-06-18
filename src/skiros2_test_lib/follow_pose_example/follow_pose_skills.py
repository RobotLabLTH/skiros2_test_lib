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

from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, Sequential, Serial, ParallelFf
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element


#################################################################################
# Description
#################################################################################

class FollowPose(SkillDescription):
    def createDescription(self):
        #=======Params=========
        #self.addParam("Container", Element(":Location"), ParamTypes.Required)
        #self.addParam("Object", Element(":Product"), ParamTypes.Optional)
        self.addParam("Pose", Element("skiros:TransformationPose"), ParamTypes.Optional)
        self.addParam("Pose2", Element("skiros:TransformationPose"), ParamTypes.Optional)

class PickAndPlace(SkillDescription):
    def createDescription(self):
        #=======Params=========
        pass

#################################################################################
# Implementation
#################################################################################

class follow_pose(SkillBase):
    """
    Tree is:
    ----->:PoseGenerator
    ----->:Skill (||)
    ------->:PoseMover
    ------->:PoseFollower

    """
    def createDescription(self):
        self.setDescription(FollowPose(), self.__class__.__name__)

    def expand(self, skill):
        skill.addChild(self.getSkill(":PoseGenerator", ""))
        parallel_node = self.getNode(ParallelFs())
        skill.addChild(parallel_node)
        parallel_node.addChild(self.getSkill(":PoseMover", "linear_mover"))
        skill.last().last().specifyParamDefault("Direction", 0)
        skill.last().addChild(self.getSkill(":PoseMover", ""))
        skill.last().last().specifyParamDefault("Direction", 1)
        skill.last().addChild(self.getSkill(":PoseFollower", ""))


class pick_and_place(SkillBase):
    """
    Tree is:
    ----->:Sequential
    ------->:PoseGenerator
    ------->:PoseGenerator
    ------->:PoseGenerator
    ----->:ParallelFf
    ------->:PoseMover, "pose_circle_mover"
    ------->:Sequential
    --------->:PoseFollower, "pose_follower_two_axis"
    --------->:PoseFollower, "pose_follower_three_axis"
    --------->:PoseFollower, "pose_follower_one_axis"
    --------->:PoseFollower, "pose_follower_three_axis"
    """
    def createDescription(self):
        self.setDescription(FollowPose(), self.__class__.__name__)

    def expand(self, skill):

        sequential_node1 = self.getNode(Sequential())
        skill.addChild(sequential_node1)

        #the pose that is going to move in circle (aka the first target)
        pose_generator1 = self.getSkill(":PoseGenerator", "")
        sequential_node1.addChild(pose_generator1)
        pose_generator1.specifyParamDefault("x", 1.)
        pose_generator1.specifyParamDefault("y", 0.)
        pose_generator1.specifyParamDefault("z", 0.)

        #the pose that represents the robot
        pose_generator2 = self.getSkill(":PoseGenerator", "")
        sequential_node1.addChild(pose_generator2)
        pose_generator2.remap("Pose", "Pose2")
        pose_generator2.specifyParamDefault("x", 1.)
        pose_generator2.specifyParamDefault("y", 1.)
        pose_generator2.specifyParamDefault("z", 1.)

        #the pose that represents the home (aka the second target)
        pose_generator3 = self.getSkill(":PoseGenerator", "")
        sequential_node1.addChild(pose_generator3)
        pose_generator3.remap("Pose", "Pose3")
        pose_generator3.specifyParamDefault("x", 0.)
        pose_generator3.specifyParamDefault("y", 2.)
        pose_generator3.specifyParamDefault("z", 2.)

        parallel_node1 = self.getNode(ParallelFf())
        skill.addChild(parallel_node1)

        pose_circle_mover = self.getSkill(":PoseMover", "pose_circle_mover")
        parallel_node1.addChild(pose_circle_mover)
        pose_circle_mover.specifyParamDefault("Direction", 2)

        sequential_node2 = self.getNode(Sequential())
        parallel_node1.addChild(sequential_node2)

        pose_follower_two_axis_1 = self.getSkill(":PoseFollowerTwoAxis", "pose_follower_two_axis")
        sequential_node2.addChild(pose_follower_two_axis_1)
        pose_follower_two_axis_1.specifyParamDefault("Axis1", 0.)
        pose_follower_two_axis_1.specifyParamDefault("Axis2", 1.)

        pose_follower_three_axis_1 = self.getSkill(":PoseFollowerThreeAxis", "pose_follower_three_axis")
        sequential_node2.addChild(pose_follower_three_axis_1)

        pose_follower_one_axis_1 = self.getSkill(":PoseFollowerOneAxis", "pose_follower_one_axis")
        sequential_node2.addChild(pose_follower_one_axis_1)
        pose_follower_one_axis_1.specifyParamDefault("Axis", 2.)
        pose_follower_one_axis_1.remap("Pose", "Pose3")

        pose_follower_three_axis_2 = self.getSkill(":PoseFollowerThreeAxis", "pose_follower_three_axis")
        sequential_node2.addChild(pose_follower_three_axis_2)
        pose_follower_three_axis_2.remap("Pose", "Pose3")
