==============================
raspi-configでの設定
==============================

::

	pi@raspberrypi ~ $ sudo raspi-config
	###青い選択画面になるので矢印キーとTabキー、Enterで次のように操作###
	* 8 Advanced Optionsを選択
	* A5 Device Treeを選択
	* Would you like the kernel to use Device Tree? で「いいえ」
	* 選択画面を出る（SPIを設定しない場合）
	###再起動（SPIを設定しない場合）###
	
	###SPIを使えるようにする###
	$ sudo raspi-config
	###青い選択画面になるので矢印キーとTabキー、Enterで次のように操作###
	* 8 Advanced Optionsを選択
	* A6 SPIを選択        * Would you like the SPI kernel module to be loaded by default?  で「はい」
	* 選択画面を出る
	###再起動###


