#!/usr/bin/python

# Copyright 2016-2020 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

state0 = state1 = state2 = 0

while 1:
    with open("/dev/rtswitch0", "r") as f:
        if f.readline() == "0\n":
            time.sleep(0.1)
            while 1:
                with open("/dev/rtswitch0", "r") as f:
                    if f.readline() != "0\n":
                        break
            time.sleep(0.1)
            state0 = (state0 + 1) & 1
            with open("/dev/rtled3", "w") as f:
                f.write(str(state0))
    with open("/dev/rtswitch1", "r") as f:
        if f.readline() == "0\n":
            time.sleep(0.1)
            while 1:
                with open("/dev/rtswitch1", "r") as f:
                    if f.readline() != "0\n":
                        break
            time.sleep(0.1)
            state1 = (state1 + 1) & 1
            with open("/dev/rtled2", "w") as f:
                f.write(str(state1))
            with open("/dev/rtled1", "w") as f:
                f.write(str(state1))
    with open("/dev/rtswitch2", "r") as f:
        if f.readline() == "0\n":
            time.sleep(0.1)
            while 1:
                with open("/dev/rtswitch2", "r") as f:
                    if f.readline() != "0\n":
                        break
            time.sleep(0.1)
            state2 = (state2 + 1) & 1
            with open("/dev/rtled0", "w") as f:
                f.write(str(state2))
