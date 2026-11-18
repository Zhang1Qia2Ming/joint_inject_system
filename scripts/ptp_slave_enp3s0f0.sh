#!/usr/bin/env bash
# 操作者不要启动 phc2sys。
# 日志出现 s2 且 offset 稳定在微秒内之前，不要设置 ptp_schedule。
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec ptp4l -i enp3s0f0 -H -s -f "${script_dir}/../config/ptp4l-enp3s0f0-slave.cfg" -m
