#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2022 RT Corporation
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

import math
import time

class Motor:
    def __init__(self, wheel_diamiter=0.048, wheel_tread=0.0925, pulse_per_rotate=400):
        # ハードウェアパラメータ
        self.wheel_diamiter = wheel_diamiter
        self.wheel_tread = wheel_tread
        self.pulse_per_rotate = pulse_per_rotate

    # デバイスファイルを通してモータの回転角度を指定
    def _set_motor_speed(self, left, right):
        try:
            with open('/dev/rtmotor_raw_l0', 'w') as lf, \
                    open('/dev/rtmotor_raw_r0', 'w') as rf:
                lf.write(str(int(left)))
                rf.write(str(int(right)))
        except Exception as e:
            print(e)

    # デバイスファイルからモータの電源をON/OFF
    def _set_motor_power(self, mode):
        try:
            with open('/dev/rtmotoren0', 'w') as f:
                f.write('1' if mode else '0')
        except Exception as e:
            print(e)

    # 車体速度（並進方向の移動速度および旋回の角速度）からモータ指令値を計算
    def _calc_speed(self, linear, angular):
        """
        linear: robot linear speed (m/s)
        angular: robot angular speed (rad/s)
        """
        left_speed = self.meter_to_pulse(linear) - self.angle_to_pulse(angular)
        right_speed = self.meter_to_pulse(linear) + self.angle_to_pulse(angular)
        print("target speed (left: {}, right: {})".format(left_speed, right_speed))
        return left_speed, right_speed

    # ラズパイマウスのモータパルスでの距離をメートル系に変換
    def pulse_to_meter(self, pulse):
        return math.pi * self.wheel_diamiter * pulse / self.pulse_per_rotate

    # メートル系での距離をラズパイマウスのモータパルスに変換
    def meter_to_pulse(self, meter):
        return int(self.pulse_per_rotate * meter / (math.pi * self.wheel_diamiter))

    # ラズパイマウスの車体の旋回角度をラズパイマウスのモータパルスに変換
    def angle_to_pulse(self, angle):
        return int(angle * self.pulse_per_rotate * self.wheel_tread / self.wheel_diamiter / (2.0 * math.pi))

    # 走行させるための関数
    def run(self, linear_speed, angular_speed):
        """
        linear_speed: m/s
        angular_speed: rad/s
        """
        (speed_l, speed_r) = self._calc_speed(linear_speed, angular_speed)

        self._set_motor_speed(speed_l, speed_r)


if __name__ == '__main__':
    motor = Motor()
    print("Motor On")
    motor._set_motor_power(1)

    print("Turn 3.14[rad/s]")
    motor.run(linear_speed=0, angular_speed=3.14)
    time.sleep(1)
    print("Turn -3.14[rad/s]")
    motor.run(linear_speed=0, angular_speed=-3.14)
    time.sleep(1)
    print("Move forward at 0.2[m/s]")
    motor.run(linear_speed=0.2, angular_speed=0)
    time.sleep(1)
    print("Move backward 0.2[m/s]")
    motor.run(linear_speed=-0.2, angular_speed=0)
    time.sleep(1)
    print("Turn 1.57[rad/s] + Move forward 0.3[m/s]")
    motor.run(linear_speed=0.3, angular_speed=1.57)
    time.sleep(1)
    print("Turn -1.57[rad/s] + Move backward 0.3[m/s]")
    motor.run(linear_speed=-0.3, angular_speed=-1.57)
    time.sleep(1)

    print("Stop")
    motor.run(linear_speed=0, angular_speed=0)
    time.sleep(0.1)

    print("Motor Off")
    motor._set_motor_power(0)
