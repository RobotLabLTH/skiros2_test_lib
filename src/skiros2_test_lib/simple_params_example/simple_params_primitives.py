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

from skiros2_skill.core.skill import SkillDescription, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import threading, Queue

#################################################################################
# Descriptions
#################################################################################

class TrajectoryGenerator(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Trajectory", dict, ParamTypes.Optional)
        self.addParam("Shutdown", False, ParamTypes.Optional)

class TrajectoryConsumer(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Trajectory", dict, ParamTypes.Required)
        self.addParam("Shutdown", bool, ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################

import random
import time

class trajectory_generator(PrimitiveBase):
    """
    This primitive generates fake "trajectories", under form of dictionaries

    To support a planning process that could take a long time, the plan function is executed in a parallel thread
    and outputs a plan every second. The execute functions just updates the parameters.

    Since parameters can be updated ONLY in the EXECUTE function, a syncronized queue is necessary
    """
    q = Queue.Queue(1)
    is_done = False
    iterations = 2

    def onStart(self):
        self.is_done = False
        self.worker = threading.Thread(target=self.plan)
        self.worker.start()
        return True

    def createDescription(self):
        self.setDescription(TrajectoryGenerator(), self.__class__.__name__)

    def plan(self):
        for i in range(0, self.iterations):
            self.q.put([random.random() for _ in xrange(5)])
            time.sleep(1)
        self.is_done = True

    def execute(self):
        if self.is_done:
            self.params["Shutdown"].setValue(True)
            return self.success("Done")
        if not self.q.empty():
            traj = {}
            traj["Trajectory"] = self.q.get()
            self.params["Trajectory"].addValue(traj)
            return self.step("Added trajectory: {}".format(self.params["Trajectory"].getValues()))
        return self.step("")

class trajectory_consumer(PrimitiveBase):
    """
    This primitive consumes one value in "trajectories" at each tick

    It continues to run until the variable Shutdown is set to True
    """
    def createDescription(self):
        self.setDescription(TrajectoryConsumer(), self.__class__.__name__)

    def execute(self):
        if self.params["Trajectory"].isSpecified():
            trajs = self.params["Trajectory"].getValues()
            traj = trajs.pop(0)
            return self.step("Consumed trajectory: {}".format(traj))
        if self.params["Shutdown"].value:
            return self.success("Done")
        return self.step("")