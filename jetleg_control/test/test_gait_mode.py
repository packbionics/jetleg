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


from jetleg_control.gait_mode import GaitMode, GaitPhase


def test_get():
    gait_phases = list()

    # Number of phases to add to gait mode
    n = 4
    for i in range(n):
        gait_phases.append(GaitPhase(0.0, 0.0, 0.0))

    gait_mode = GaitMode(gait_phases=gait_phases)

    for i in range(n):
        assert gait_mode.get(i) is not None

    try:
        gait_mode.get(-1)
        assert False
    except ValueError as e:
        assert str(e) == "Invalid index given for gait phase: -1"

    try:
        gait_mode.get(4)
        assert False
    except ValueError as e:
        assert str(e) == "Invalid index given for gait phase: 4"
