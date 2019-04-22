#!/usr/bin/env bash
set -u
set +exv

echo "==================="
echo "ERROR: $@"
echo "If you need someone's support, you should share this information."
uname -a
lsb_release -a || cat /etc/lsb-release
ls /usr/src/linux*
echo "==================="
