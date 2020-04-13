# RaspberryPiMouse

[![Build Status](https://travis-ci.org/rt-net/RaspberryPiMouse.svg?branch=master)](https://travis-ci.org/rt-net/RaspberryPiMouse)

This repository has the source code and kernel objects
for the Raspberry Pi Mouse.

## インストール

インストール用のシェルスクリプト（[`./utils/build_install.bash`](https://github.com/rt-net/RaspberryPiMouse/blob/master/utils/build_install.bash)）を実行します。

### Raspbianの場合

```sh
$ git clone https://github.com/rt-net/RaspberryPiMouse.git
$ cd RaspberryPiMouse/utils
$ sudo apt install raspberrypi-kernel-headers
$ ./build_install.bash
```

### Ubuntuの場合

```sh
$ git clone https://github.com/rt-net/RaspberryPiMouse.git
$ cd RaspberryPiMouse/utils
$ sudo apt install linux-headers-$(uname -r)
$ ./build_install.bash
```

## マニュアルインストール

```sh
$ git clone https://github.com/rt-net/RaspberryPiMouse.git
$ cd RaspberryPiMouse/src/drivers
$ make
$ sudo insmod rtmouse.ko
```

## ドライバの導入の際の注意

### Raspbian

以下の設定を確認ください。
`raspi-config` コマンドで設定します。

* SPI機能を「入」にする。
* I2C機能を「入」にする。

2017年1月現在、以下の設定は不要です。  
rtmouseをインストールして不具合が出た場合のみ以下の設定を追加で行ってください。

* Device Tree機能を「切」にする。

### arm64版Ubuntu18.04

I2Cのbaudrateをデフォルト値より下げる必要があります（[issues#13](https://github.com/rt-net/RaspberryPiMouse/issues/13)）。

`/boot/firmware/config.txt`に以下の1行を追加することでI2Cのbaudrateを62.5kHzに固定することができます。

```
dtparam=i2c_baudrate=62500
```

### Raspberry Pi 4

Raspberry Pi 4ではCPUのレジスタがそれまでのRaspberry Piとは異なります（[issues#21](https://github.com/rt-net/RaspberryPiMouse/issues/21)）。  
Raspberry Pi 4で本ドライバを使用する際には`rtmouse.c`の以下の行（2020年4月13日現在の最新版のv2.1.0では[54行目](https://github.com/rt-net/RaspberryPiMouse/blob/dd0343449951a99b067e24aef3c03ae5ed9ab936/src/drivers/rtmouse.c#L54)）を`RASPBERRYPI 4`に書き換えて手動でビルドする必要があります。

```c
// define the Raspberry Pi version here
// Raspberry Pi 1 B/A/B+/A+: 1
// Raspberry Pi 2 B        : 2
// Raspberry Pi 3 B/A+/B+  : 2
// Raspberry Pi 4 B        : 4
#define RASPBERRYPI 2
```

### その他

その他のよくある質問については[wiki](https://github.com/rt-net/RaspberryPiMouse/wiki)にまとめています。

## 日経Linux連載

連載（Raspberry Piで始めるかんたんロボット製作）で上田氏が書いた
シェルスクリプトは下記にあります。

https://github.com/ryuichiueda/RPiM


## License

This repository is licensed under the GPLv3 License, see [LICENSE](./LICENSE).

このリポジトリはGPLv3ライセンスで公開されています。詳細は[LICENSE](./LICENSE)を確認してください。

### Includings

This repository contains the code of the repository shown below.

このリポジトリは以下に示すリポジトリのコードを一部含みます。

* [take-iwiw/DeviceDriverLesson](https://github.com/take-iwiw/DeviceDriverLesson)
  * GPL/BSD License
* [mcp3204.c in Raspberry Piで学ぶARMデバイスドライバープログラミング](http://www.socym.co.jp/support/s-940#ttlDownload)
  * GPL v2 License
* [RPi-Distro/raspi-gpio](https://github.com/RPi-Distro/raspi-gpio)
  * The 3-Clause BSD License
