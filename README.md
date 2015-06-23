# RaspberryPiMouse

This repository has the source code and kernel objects
for the Raspberry Pi mouse articles in Nikkei Linux　2015.

Shell scripts to use these drivers are placed in https://github.com/ryuichiueda/RPiM

日経Linux連載用ドライバー readme
上田氏が書いてくれているシェルスクリプトは下記にあります。
https://github.com/ryuichiueda/RPiM

## ドライバの導入の際の注意

以下の設定を確認ください。
raspi-configコマンドで設定します。

* SPI機能を「入」にする。
* Device Tree機能を「切」にする。
