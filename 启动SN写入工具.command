#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"
echo "=================================================="
echo " 正在启动 中岳航空飞控 20位 SN 码产线写入工具 ..."
echo "=================================================="
python3 sn_writer_gui.py
