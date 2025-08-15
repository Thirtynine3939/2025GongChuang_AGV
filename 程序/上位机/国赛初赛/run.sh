#!/bin/bash
# 获取脚本所在目录并切换到该目录
cd "$(dirname "$0")" || exit
# 运行main.py（使用python3）
/usr/bin/python main.py
