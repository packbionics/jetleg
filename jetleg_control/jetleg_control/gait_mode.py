# Copyright 2023 Pack Bionics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


from typing import List
from dataclasses import dataclass


@dataclass
class GaitPhase:
    stiffness: List[float]
    damping: List[float]
    equilibrium: List[float]


class GaitMode:
    """Describe a mostly closed cycle of repeated gait phases."""

    def __init__(self, gait_phases: List[GaitPhase]):
        """Construct a Gait Mode with an ordered list of phases."""
        self.gait_phases = gait_phases

    def get(self, idx: int) -> GaitPhase:
        """Retrieve the gait phase at the given index."""
        if idx < 0 or idx >= len(self.gait_phases):
            raise ValueError(f"Invalid index given for gait phase: {idx}")

        return self.gait_phases[idx]
