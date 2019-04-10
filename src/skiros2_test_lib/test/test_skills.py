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

from skiros2_skill.core.skill import SkillDescription, SkillBase, Selector, State, ParallelFf, Sequential
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
    """
    Tree is:
    ----->: ()
    ------->:
    ------->:

    """

    def createDescription(self):
        self.setDescription(TestSkill(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill(":TestPrimitive", ""),
            self.skill(":TestPrimitive", "", specify={"Force": 1.0})
        )


class test_skill_parallel(SkillBase):
    """
    Tree is:
    ----->: ()
    ------->:
    ------->:

    """

    def createDescription(self):
        self.setDescription(TestSkill(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFf())
        skill(
            self.skill(":TestPrimitive", "test_primitive"),
            self.skill(":TestPrimitive", "test_primitive", specify={"Force": 1.0}),
            self.skill(":TestPrimitive", "", specify={"Force": 2.0})
        )
