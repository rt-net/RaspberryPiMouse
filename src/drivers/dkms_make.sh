#!/usr/bin/env bash
set -euo pipefail

kernel_version=${1:-$(uname -r)}
mode=${2:-build}
script_dir=$(cd "$(dirname "${BASH_SOURCE:-$0}")" && pwd)

# Raspberry Pi 4 はベースアドレスが異なるため RASPBERRYPI=4 でビルドする。
# Raspberry Pi 3 は既存実装と同じく RASPBERRYPI=2 として扱う。
detect_raspberrypi_version() {
	if grep -q "Raspberry Pi 4" /proc/cpuinfo; then
		printf '4\n'
		return
	fi

	if grep -q "Raspberry Pi" /proc/cpuinfo; then
		printf '2\n'
		return
	fi

	printf '2\n'
	printf 'dkms_make.sh: Raspberry Pi model was not detected; building with RASPBERRYPI=2\n' >&2
}

raspberrypi_version=$(detect_raspberrypi_version)

# DKMS は /usr/src/rtmouse-1.0 から実行するため、ドライバディレクトリへ移動する。
cd "$script_dir"

# apt で入れた実行中カーネルのヘッダーを使う Makefile を選ぶ。
ln -sf Makefile.header_from_apt Makefile

# ヘッダー削除後の dkms remove でも clean が失敗しないよう、生成物だけ消す。
if [ "$mode" = "clean" ] && [ ! -e "/lib/modules/$kernel_version/build" ]; then
	rm -f ./*.o ./*.ko ./*.mod ./*.mod.c ./*.cmd Module.symvers modules.order
	rm -rf .tmp_versions
	exit 0
fi

# 古い生成物を消してから、DKMS が指定したカーネルバージョン向けにビルドする。
make clean KVERSION="$kernel_version" RASPBERRYPI="$raspberrypi_version"

if [ "$mode" = "clean" ]; then
	exit 0
fi

make KVERSION="$kernel_version" RASPBERRYPI="$raspberrypi_version"
