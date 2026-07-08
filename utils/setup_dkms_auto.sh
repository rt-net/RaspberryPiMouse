#!/usr/bin/env bash
set -eu

PACKAGE_NAME=rtmouse
PACKAGE_VERSION=1.0
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE:-$0}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
DKMS_SRC_DIR="/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}"

kernel_version_int() {
	echo "$1" | awk -F. '{ printf "%d%02d", $1, $2 }'
}

KERNEL_VERSION=$(uname -r | cut -d'-' -f1)
KERNEL_VERSION_INT=$(kernel_version_int "$KERNEL_VERSION")
USE_MODULES_LOAD_CONF=0
if [ "$KERNEL_VERSION_INT" -lt "$(kernel_version_int 5.16)" ]; then
	USE_MODULES_LOAD_CONF=1
fi

# sudo 経由でも root 直実行でも同じコマンド列で動かすための準備。
if [ "$(id -u)" -eq 0 ]; then
	SUDO=
else
	SUDO=sudo
fi

# 対象OSだけを許可し、DKMS ビルドに必要な最低限のパッケージを入れる。
. /etc/os-release
case "${ID:-}" in
	ubuntu | raspbian)
		;;
	debian)
		if [ ! -r /etc/rpi-issue ] && ! echo "${ID_LIKE:-}" | grep -qi "raspbian"; then
			echo "Unsupported OS: ${PRETTY_NAME:-unknown}" >&2
			exit 1
		fi
		;;
	*)
		echo "Unsupported OS: ${PRETTY_NAME:-unknown}" >&2
		exit 1
		;;
esac
$SUDO apt-get update
$SUDO apt-get install -y "linux-headers-$(uname -r)" build-essential dkms

# いま起動しているカーネル用のヘッダーが無い場合、DKMS ビルドはできない。
# apt によるカーネル更新直後は、再起動してから再実行する必要がある。
if [ ! -e "/lib/modules/$(uname -r)/build" ]; then
	echo "Kernel headers for the running kernel were not found." >&2
	echo "Please reboot once if apt installed a new Raspberry Pi kernel, then rerun this script." >&2
	exit 1
fi

# 以前の手動インストールや古い DKMS 登録があると衝突するため、先に取り除く。
if dkms status "$PACKAGE_NAME" 2>/dev/null | grep -q "^${PACKAGE_NAME}/${PACKAGE_VERSION},"; then
	$SUDO dkms remove -m "$PACKAGE_NAME" -v "$PACKAGE_VERSION" --all
fi
$SUDO rm -f "/lib/modules/$(uname -r)/kernel/drivers/misc/${PACKAGE_NAME}.ko"
$SUDO rm -f "/etc/modules-load.d/${PACKAGE_NAME}.conf"

# デバイスファイル作成時に権限付与とモーター無効化を行う udev ルールを配置する。
$SUDO install -m 0644 "$REPO_ROOT/50-rtmouse.rules" /etc/udev/rules.d/50-rtmouse.rules
$SUDO udevadm control --reload-rules
$SUDO udevadm trigger --action=add || true

# DKMS がカーネル更新時にも同じソースを参照できるように /usr/src へ配置する。
$SUDO rm -rf "$DKMS_SRC_DIR"
$SUDO mkdir -p "$DKMS_SRC_DIR"
$SUDO cp -a "$REPO_ROOT/." "$DKMS_SRC_DIR/"
$SUDO rm -rf "$DKMS_SRC_DIR/.git"
$SUDO chmod +x "$DKMS_SRC_DIR/src/drivers/dkms_make.sh"

# rtmouse を DKMS に登録し、現在のカーネル向けにビルド・インストールする。
$SUDO dkms add -m "$PACKAGE_NAME" -v "$PACKAGE_VERSION"
$SUDO dkms build -m "$PACKAGE_NAME" -v "$PACKAGE_VERSION" -k "$(uname -r)"
$SUDO dkms install -m "$PACKAGE_NAME" -v "$PACKAGE_VERSION" -k "$(uname -r)"

# DKMS で入れたモジュールを現在のシステムから参照できるようにする。
$SUDO depmod -a

# カーネル 5.16 未満では device tree 経由でロードされないため明示的に自動ロードする。
if [ "$USE_MODULES_LOAD_CONF" -eq 1 ]; then
	printf '%s\n' "$PACKAGE_NAME" | $SUDO tee "/etc/modules-load.d/${PACKAGE_NAME}.conf" > /dev/null
fi

# 現在のセッションでのmodprobe実行
$SUDO modprobe "$PACKAGE_NAME"
