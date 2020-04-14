# RaspberryPiMouse

![CI](https://github.com/rt-net/RaspberryPiMouse/workflows/CI/badge.svg?branch=master)

This repository has the source code and kernel objects
for the Raspberry Pi Mouse.

## インストール

./utilディレクトリのシェルスクリプトを実行します。

```
$ git clone https://github.com/rt-net/RaspberryPiMouse.git
$ cd utils
###Raspbianの場合###
$ sudo apt install raspberrypi-kernel-headers
$ ./build_install.bash
###Ubuntuの場合###
$ sudo apt install linux-headers-$(uname -r)
$ ./build_install.bash
```


## How to install the device driver（マニュアルインストール）

```
$ git clone https://github.com/rt-net/RaspberryPiMouse.git
### check the kernel version
$ uname -r
4.1.6-v7+
###choose a directory based on your RPi and the kernel version
$ cd RaspberryPiMouse/lib/Pi2B+/4.1.6-v7+/
###install the kernel object
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
