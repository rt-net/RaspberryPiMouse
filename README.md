# RaspberryPiMouse

[![CI](https://github.com/rt-net/RaspberryPiMouse/actions/workflows/driver-cross-build.yml/badge.svg)](https://github.com/rt-net/RaspberryPiMouse/actions/workflows/driver-cross-build.yml)

This repository has the source code and kernel objects
for the Raspberry Pi Mouse.

## Installation

Run the following scripts.

- setting script ([`./utils/set_configs.bash`](https://github.com/rt-net/RaspberryPiMouse/blob/master/utils/set_configs.bash))
- installation script ([`./utils/build_install.bash`](https://github.com/rt-net/RaspberryPiMouse/blob/master/utils/build_install.bash))

以下のスクリプトを実行します
- setting script（[`./utils/set_configs.bash`](https://github.com/rt-net/RaspberryPiMouse/blob/master/utils/set_configs.bash)）
- installation script（[`./utils/build_install.bash`](https://github.com/rt-net/RaspberryPiMouse/blob/master/utils/build_install.bash)）

### for `Raspberry Pi OS`・`Ubuntu`

以下のコマンドで本リポジトリをダウンロードし、Raspberry Pi本体の設定を行います。

```bash
$ git clone https://github.com/rt-net/RaspberryPiMouse.git
$ cd RaspberryPiMouse/utils
$ ./set_configs.bash
```

Raspberry Piを再起動し、以下のコマンドを実行してビルドに必要なファイルをインストールします。**`Ubuntu`と`Raspberry Pi OS`でコマンドが違います。**

```bash
# Ubuntu Serverの場合
$ sudo apt install linux-headers-$(uname -r) build-essential

# Raspberry Pi OSの場合
$ sudo apt install raspberrypi-kernel-headers build-essential
```

以下のコマンドでビルドを実行します。

```bash
$ cd RaspberryPiMouse/utils
$ ./build_install.bash
```

### Manual installation

```bash
$ git clone https://github.com/rt-net/RaspberryPiMouse.git
$ cd RaspberryPiMouse/src/drivers
$ make
$ sudo insmod rtmouse.ko
```

## Notes for the installation (ドライバの導入の際の注意)

### for Raspberry Pi OS

以下の設定を確認ください。※[`./utils/set_configs.bash`](https://github.com/rt-net/RaspberryPiMouse/blob/master/utils/set_configs.bash)を実行すると、設定は[自動で書き換わります]((https://github.com/rt-net/RaspberryPiMouse/blob/master/utils/set_configs.bash#L51-#L61))。

#### for SPI and I2C

Enable SPI and I2C functions via `raspi-config` command.

`raspi-config` コマンドで設定します。

* SPI機能を「入」にする。
* I2C機能を「入」にする。

#### for 32-bit OS

Set 32bit-setting to `/boot/firmware/config.txt`.

32-bit版のOSではビルドするために、`/boot/firmware/config.txt`に以下の1行を追加する必要があります。

```bash
arm_64bit=0
```

### for Raspberry Pi 4

Edit [`rtmouse.c`](https://github.com/rt-net/RaspberryPiMouse/blob/dd0343449951a99b067e24aef3c03ae5ed9ab936/src/drivers/rtmouse.c#L54) to change the defined value `RASPBERRYPI` from '2' to '4'.

Raspberry Pi 4ではCPUのレジスタがそれまでのRaspberry Piとは異なります（[issues#21](https://github.com/rt-net/RaspberryPiMouse/issues/21)）。
Raspberry Pi 4で本ドライバを使用する際には`rtmouse.c`の以下の行（2020年4月13日現在の最新版のv2.1.0では[54行目](https://github.com/rt-net/RaspberryPiMouse/blob/dd0343449951a99b067e24aef3c03ae5ed9ab936/src/drivers/rtmouse.c#L54)）を`RASPBERRYPI 4`に書き換えてビルドする必要があります。
※[`./utils/build_install.bash`](./utils/build_install.bash)を実行すると、Raspberry Piのモデルに合わせて[`rtmouse.c`](./src/drivers/rtmouse.c)が[自動で書き換わります](https://github.com/rt-net/RaspberryPiMouse/blob/a9af4fa2b2a8e34c0f93a6ce5cf88ebd50ff39c2/utils/build_install.raspi4ubuntu.bash#L13-L14)。

```c
// define the Raspberry Pi version here
// Raspberry Pi 1 B/A/B+/A+: 1
// Raspberry Pi 2 B        : 2
// Raspberry Pi 3 B/A+/B+  : 2
// Raspberry Pi 4 B        : 4
#define RASPBERRYPI 2
```

### デバイスツリーオーバーレイについて

kernel `5.16`以降では`/boot/firmware/config.txt`に以下の設定を記述し、dtoverlayを設定する必要があります。※[`./utils/set_configs.bash`](https://github.com/rt-net/RaspberryPiMouse/blob/master/utils/set_configs.bash)を実行すると、設定は[自動で書き換わります]((https://github.com/rt-net/RaspberryPiMouse/blob/master/utils/set_configs.bash#L35-#L49))。

```bash
dtoverlay=anyspi:spi0-0,dev="microchip,mcp3204",speed=1000000
```

### パルスカウンタについて

パルスカウンタは値の読み取りにI2Cを使用しています。仕様上は400kHzまでbaudrateを上げることができます（※1）。
I2Cのbaudrateを上げると通信に失敗する場合がある（[issues#13](https://github.com/rt-net/RaspberryPiMouse/issues/13)）ので、基本的にはI2Cのbaudrateはデフォルト値（※2）から変更して62.5kHzに固定してください。

According to
[issues#13](https://github.com/rt-net/RaspberryPiMouse/issues/13),
it may be necessary to set the I2C baudrate lower than the default value.
Add a following new line in `/boot/firmware/config.txt` to change the i2c_baudrate to 62.5 kHz.

`/boot/firmware/config.txt`に以下の1行を追加することでI2Cのbaudrateを62.5kHzに固定することができます。

```bash
dtparam=i2c_baudrate=62500
```

※1　Raspberry Pi 4 Model B（Ubuntu Server `18.04` / `20.04` / `22.04` / `24.04`）を搭載して400kHzで通信できることを確認しています。
※2　現在設定されているI2Cのbaudrateは以下のコマンドを実行することで確認できます。
```
$ printf "%d\n" 0x$(xxd -ps /sys/class/i2c-adapter/i2c-1/of_node/clock-frequency)
```

## Device files

For example code of device files, please refer to [SampleProgram](./SampleProgram/README.md).

デバイスファイルの使用例は[サンプルプログラム](./SampleProgram/README.md)を参考にしてください。

### Light sensor x4 (Input)

Read `/dev/rtlightsensor0` to get proximity (0:far ~ 4095:close) of objects detected by light sensors.

`/dev/rtlightsensor0`を読み取り、光センサで検出された物体の近接度 (0:遠い ~ 4095:近い)を取得します。

```sh
# cat /dev/rtlightsensor0
# Return value: [front right] [right] [left] [front left]
$ cat /dev/rtlightsensor0
9 2 13 3
```

### Switch x3 (Input)

Read `/dev/rtswitch0` ~ `/dev/rtswitch2` to get the switches on/off state.

`/dev/rtswitch0` ~ `/dev/rtswitch2` を読み取りスイッチのON/OFF状態を取得します。

```sh
# cat /dev/rtswitch[0,1]
# Return value: 1(Open), 0(Pressed)
$ cat /dev/rtswitch0
```

### Buzzer (Output)

Write 0 ~ 20000 to `/dev/rtbuzzer0` to beep the buzzer.

`/dev/rtbuzzer0` に0 ~ 20000を書き込みブザーを鳴らします。

```sh
# echo 0 ~ 20000(Hz) > /dev/rtbuzzer0
$ echo 440 > /dev/rtbuzzer0
$ echo 0 > /dev/rtbuzzer0
```

### LED x4 (Output)

Write 1/0 to `/dev/rtled0` ~ `/dev/rtled3` to turn on/off the LEDs.

`/dev/rtled0` ~ `/dev/rtled3` に1/0を書き込みLEDを点灯/消灯します。

```sh
# echo 0(OFF) or 1(ON) > /dev/rtled[0,1,2,3]
$ echo 1 > /dev/rtled0
$ echo 0 > /dev/rtled1
```

### Motor enable (Output)

Write 1/0 to `/dev/rtmotoren0` to enable/disable motors control.

`/dev/rtmotoren0` に 1/0 を書き込みモータ操作を有効/無効にします。

```sh
# echo 0(disable) or 1(enable) > /dev/rtmotoren0
$ echo 1 > /dev/rtmotoren0
```

### PWM frequency for left/right motor driver (Output)

Write 0 ~ ±10000 to `/dev/rtmotor_raw_l0` or `/dev/rtmotor_raw_r0` to set PWM frequency for motor drivers.

※ 0 ~ ±4 Hz will be reset to 0 Hz.

`/dev/rtmotor_raw_l0` または `/dev/rtmotor_raw_r0` に 0 ~ ±10000 を書き込み、モータドライバへのPWM周波数を設定します。

※ 0 ~ ±4 Hzは0Hzへリセットされます

```sh
# echo 0 ~ ±10000(Hz) > /dev/rtmotor_raw_[l0, r0]
$ echo 1 > /dev/rtmotoren0
$ echo 400 > /dev/rtmotor_raw_l0
```

### PWM frequencies and drive duration (Output)

Write left and right PWM frequencies and drive duration to `/dev/rtmotor0` to drive both motors.

`/dev/rtmotor0`に左右のPWM周波数と動作時間を書き込み、左右のモータを回します。

```sh
# echo [left_freq Hz] [right_freq Hz] [duration ms] > /dev/rtmotor0
$ echo 1 > /dev/rtmotoren0
$ echo 400 800 1000 > /dev/rtmotor0
```

### Pulse counter x2 (Input/Output)

Read `/dev/rtcounter_*` to get pulse counts of PWM for motor drivers or write values to reset counts.

`/dev/rtcounter_*`を読み取りモータドライバへのPWMパルス数を取得します。また、値を書き込みカウントをリセットします。

- unsigned counters : `/dev/rtcounter_l0`, `/dev/rtcounter_r0`
- signed counters : `/dev/rtcounter_l1`, `/dev/rtcounter_r1`

```sh
# cat /dev/rtcounter_[l0, r0]
# Return value: 0 ~ 65565 (counts of PWM pulse)
# cat /dev/rtcounter_[l1, r1]
# Return value: -32767 ~ 32767 (counts of PWM pulse)
$ cat /dev/rtcounter_l0
1104
$ echo 0 > /dev/rtcounter_l0
$ cat /dev/rtcounter_l0
0
```

## その他

その他のよくある質問については[wiki](https://github.com/rt-net/RaspberryPiMouse/wiki#%E3%82%88%E3%81%8F%E3%81%82%E3%82%8B%E8%B3%AA%E5%95%8F)にまとめています。

## 日経Linux連載

連載（Raspberry Piで始めるかんたんロボット製作）で上田氏が書いた
シェルスクリプトは下記にあります。

https://github.com/ryuichiueda/RPiM


## License

This repository except for `SampleProgram` directory is licensed under the [GPL-2.0-only](https://spdx.org/licenses/GPL-2.0-only.html) License, see [LICENSE](./LICENSE).

`SampleProgram` directory is licensed under the Apache 2.0 License, see [SampleProgram / LICENSE](./SampleProgram/LICENSE).

このリポジトリは`SampleProgram`ディレクトリを除き[GPL-2.0-only](https://spdx.org/licenses/GPL-2.0-only.html)ライセンスで公開されています。詳細は[LICENSE](./LICENSE)を確認してください。

`SampleProgram`ディレクトリは[Apache-2.0](https://spdx.org/licenses/Apache-2.0.html)ライセンスで公開されています。詳細は[SampleProgram / LICENSE](./SampleProgram/LICENSE)を確認してください。

### Includings

This repository contains the code of the repository shown below.

このリポジトリは以下に示すリポジトリのコードを一部含みます。

* [take-iwiw/DeviceDriverLesson](https://github.com/take-iwiw/DeviceDriverLesson)
  * GPL/BSD License
* [mcp3204.c in Raspberry Piで学ぶARMデバイスドライバープログラミング](http://www.socym.co.jp/support/s-940#ttlDownload)
  * GPL v2 License
* [RPi-Distro/raspi-gpio](https://github.com/RPi-Distro/raspi-gpio)
  * The 3-Clause BSD License
