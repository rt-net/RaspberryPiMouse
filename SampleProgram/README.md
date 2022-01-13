# Sample Program

このディレクトリにはLED、スイッチ、ブザーなどのRaspberryPiMouseの周辺機器を使うサンプルコードがあります。

This directory contains examples how to use RaspberryPiMouse's peripherals like LEDs, switches, buzzer, etc.

このディレクトリはApache 2.0ライセンスで公開されています。詳細は[SampleProgram / LICENSE](./LICENSE)を確認してください。

This directory is licensed under the Apache 2.0 License, see [SampleProgram / LICENSE](./LICENSE).


# How To Use

各機能ごとにShell Script、C、Pythonで書かれたサンプルコードを用意しています。

Each peripheral example has sample code files written Shell Script, C and Python.

RT Software Tutorialsでは動画つきで説明しています。

https://rt-net.github.io/tutorials/raspimouse/driver/samples.html

## Shell Script

```sh
$ bash step1.sh

Ctrl-c を押して終了してください。
Press [Ctrl-c] to terminate.
```

## C

サンプル実行前に、gccでコンパイルします。

Compile an example code via gcc command before execution.

```sh
$ gcc step1.c -o step1
$ ./step1

Ctrl-c を押して終了してください。
Press [Ctrl-c] to terminate.
```

## Python

```sh
$ python3 step1.py

Ctrl-c を押して終了してください。
Press [Ctrl-c] to terminate.
```

# Step1

LED0~LED3が点滅します。

The LED0 ~ LED3 blinks.

# Step2

キーボード入力でブザーを鳴らします。
入力キーと音階のペアは[./SCALE](./SCALE)を参照してください。

'0'を入力するとブザーが止まります。
'c'を入力するとプログラムが終了します。（Shell ScriptではCtrl-cで終了します。）

Beep the buzzer with keyboard input. 
Refer to the [./SCALE](./SCALE) file for the pairs of input characters and scales.

Press '0' to stop beeping.
Press 'c' to terminate the program. (Press Ctrl-c for the Shell Script.)

# Step3

スイッチを押してLEDを点消灯します。
組み合わせは次のとおりです。

Turn on/off LEDs by the switches.
The combination of LEDs and switches are defined as bellow.

- SW0 : LED3
- SW1 : LED1 & LED2
- SW2 : LED0

# Step4

モータを回して右旋回、左旋回します。

Drive the motors to rotate clockwise and counter-clockwise.

# Step5

ライトセンサの値を読み込みます。

Read the lightsensors values.

# Step6

モータを回して、パルスカウンタの値を読み込みます。

Drive the motors and read the pulse counters values.

# Step7

車体速度![v_{fw}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+v_%7Bfw%7D)、
車体角速度![v_{rot}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+v_%7Brot%7D)で指令するサンプルプログラムです。

サンプルでは車体速度と車体角速度を指定しての移動が実装されています。

## 解説

ホイールの直径を![\phi](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cphi)[m]、車体のトレッドを![t](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+t)[m]、モータ1回転のための制御信号を![p](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+p)[Hz]とします。

左右のモータへの制御信号を![\omega_{rot}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Comega_%7Brot%7D)[Hz]で入力したときの、
車体の並進方向の速度を![v_{fw}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+v_%7Bfw%7D)[m/s]とします。  
このときのそれぞれの関係は以下のように表現できます。

![v_{fw} : \omega_{fw} = \pi \phi  : p](https://render.githubusercontent.com/render/math?math=%5Clarge+%5Cdisplaystyle+v_%7Bfw%7D+%3A+%5Comega_%7Bfw%7D+%3D+%5Cpi+%5Cphi++%3A+p)

旋回方向についても同様に考えます。  
車体が1回転するときのホイールが円弧を描くように移動する距離は![\pi t](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cpi+t)[m]で計算できます。
モータ1回転でホイールの表面が移動する距離は![\pi \phi](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cpi+%5Cphi)[m]で計算できます。  
左右のモータへの制御信号をそれぞれ![-\omega_{fw}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+-%5Comega_%7Bfw%7D)[Hz]と![\omega_{fw}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Comega_%7Bfw%7D)[Hz]としたとき、
ロボットが旋回するときの角速度を![v_{rot}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+v_%7Brot%7D)[rad/s]とします。  
このときのそれぞれの関係は以下のように表現できます。

![v_{rot} : \omega_{rot} = \frac{2\pi} {\pi t / \pi \phi}  : p](https://render.githubusercontent.com/render/math?math=%5Clarge+%5Cdisplaystyle+v_%7Brot%7D+%3A+%5Comega_%7Brot%7D+%3D+%5Cfrac%7B2%5Cpi%7D+%7B%5Cpi+t+%2F+%5Cpi+%5Cphi%7D++%3A+p)

これらを整理するとモータ制御信号は以下の2つの式にまとめられます。

![\omega_{fw} = \frac{p} {\pi \phi} v_{fw} ](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Comega_%7Bfw%7D+%3D+%5Cfrac%7Bp%7D+%7B%5Cpi+%5Cphi%7D+v_%7Bfw%7D+)

![\omega_{rot} = \frac {tp}{2\pi \phi} v_{rot}](https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Comega_%7Brot%7D+%3D+%5Cfrac+%7Btp%7D%7B2%5Cpi+%5Cphi%7D+v_%7Brot%7D)
