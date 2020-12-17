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
import sys

files = ["/dev/rtled0", "/dev/rtled1", "/dev/rtled2", "/dev/rtled3"]

while 1:
    for filename in files:
        with open(filename, 'w') as f:
            f.write("1")
    time.sleep(0.5)
    for filename in files:
        with open(filename, 'w') as f:
            f.write("0")
    time.sleep(0.5)
