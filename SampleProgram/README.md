# Sample Program

このディレクトリにはLED、スイッチ、ブザーなどのRaspberryPiMouseの周辺機器を使うサンプルコードがあります。

This directory contains examples how to use RaspberryPiMouse's peripherals like LEDs, switches, buzzer, etc.

# How To Use

各機能ごとにShell Script、C、Pythonで書かれたサンプルコードを用意しています。

Each peripheral example has sample code files written Shell Script, C and Python.

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
Refer the [./SCALE](./SCALE) file for the pairs of input characters and scales.

Press '0' to stop beeping.
Press 'c' to terminate the program. (Press Ctrl-c for the Shell Script.)

# Step3

スイッチを押してLEDを点消灯します。
組み合わせは次のとおりです。

Turn on/off LEDs by the switches.
The pairs of LEDs and switches are bellow:

- SW0 : LED3
- SW1 : LED1 & LED2
- SW2 : LED0
