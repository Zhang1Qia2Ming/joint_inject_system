#!/usr/bin/env bash
# 连续抓 PL 里的 D10 上升沿。不烧录比特流。
# 用法：./capture_d10_ila.sh        一直抓，Ctrl-C 结束
#       ./capture_d10_ila.sh 20     抓满 20 个上升沿后结束
set -euo pipefail

COUNT="${1:-0}"
if [[ ! "$COUNT" =~ ^[0-9]+$ ]]; then
    printf '用法：%s [上升沿次数，省略则一直抓]\n' "$0" >&2
    exit 1
fi

LTX=/data/work/gmsl_4ev/code/mipi_gmsl/zu4_pcie_gmsl_tx1rx1_6gloop/project_1.runs/impl_uyvy/design_uyvy_wrapper.ltx
VIVADO=/data/xilinx/Vivado/2020.2/bin/vivado
TCL=$(mktemp /tmp/capture_d10_ila.XXXXXX.tcl)
cleanup() { rm -f "$TCL"; }
trap cleanup EXIT

cat >"$TCL" <<EOF
set capture_count $COUNT
set ltx_file {$LTX}
set csv_file /tmp/d10_ila_live.csv

fconfigure stdout -buffering line
open_hw_manager
connect_hw_server -url 127.0.0.1:3121
open_hw_target

set dev [lindex [get_hw_devices xczu4_0] 0]
if {\$dev eq ""} {
    set dev [lindex [get_hw_devices] 0]
}
current_hw_device \$dev
set_property PROBES.FILE \$ltx_file \$dev
set_property FULL_PROBES.FILE \$ltx_file \$dev
refresh_hw_device \$dev

set ila [lindex [get_hw_ilas -of_objects \$dev] 0]
if {\$ila eq ""} {
    puts "当前比特流里没有 ILA，抓不到 D10"
    close_hw_manager
    exit 2
}
set edge_probe ""
foreach probe [get_hw_probes -of_objects \$ila] {
    if {[get_property NAME \$probe] eq "design_uyvy_i/mipi_gen_0_dbg_bus"} {
        set edge_probe \$probe
    }
}
if {\$edge_probe eq ""} {
    puts "ILA 里没有 mipi_gen_0_dbg_bus，这份探针文件和正在运行的比特流不一致"
    close_hw_manager
    exit 3
}

proc arm_d10 {ila edge_probe} {
    reset_hw_ila \$ila
    set_property CONTROL.TRIGGER_POSITION 512 \$ila
    foreach probe [get_hw_probes -of_objects \$ila] {
        set width [get_property WIDTH \$probe]
        set_property TRIGGER_COMPARE_VALUE eq\${width}'b[string repeat X \$width] \$probe
    }
    set_property TRIGGER_COMPARE_VALUE eq48'b[string repeat X 46]1X \$edge_probe
    run_hw_ila \$ila
}

proc read_edge {csv_file} {
    set fh [open \$csv_file r]
    set previous ""
    set hit ""
    while {[gets \$fh line] >= 0} {
        set line [string trim \$line]
        if {\$line eq "" || [string match Sample* \$line] || [string match Radix* \$line]} {
            continue
        }
        set fields [split \$line ,]
        if {[llength \$fields] < 4} continue
        set sample [string trim [lindex \$fields 0]]
        set trigger [string trim [lindex \$fields 2]]
        set bus_text [string trim [lindex \$fields end]]
        if {![string is integer -strict \$sample]} continue
        set bus [scan \$bus_text %x]
        if {\$trigger eq "1"} {
            set hit [list \$sample \$previous \$bus]
            break
        }
        set previous \$bus
    }
    close \$fh
    return \$hit
}

puts "已连上 ILA。触发条件是 D10 上升沿脉冲 dbg_bus\[1\]。"
puts "每次窗口 1024 拍、150 MHz，大约 6.8 µs。打印间隔是 JTAG 重新武装的时间，不是 D10 的周期。"
if {\$capture_count == 0} {
    puts "一直抓，Ctrl-C 结束。"
} else {
    puts "抓满 \$capture_count 个上升沿后结束。"
}

set seen 0
set previous_ms ""
while {\$capture_count == 0 || \$seen < \$capture_count} {
    set started [clock milliseconds]
    if {[catch {arm_d10 \$ila \$edge_probe} arm_error]} {
        puts "武装 ILA 失败：\$arm_error"
        after 200
        continue
    }
    set wait_error ""
    if {[catch {wait_on_hw_ila -timeout 2000 \$ila} wait_error]} {
        puts "[clock format [clock seconds] -format {%H:%M:%S}] 2 秒内没有采到 D10 上升沿"
        continue
    }
    set data [upload_hw_ila_data \$ila]
    write_hw_ila_data -force -csv_file \$csv_file \$data
    set hit [read_edge \$csv_file]
    set now_ms [clock milliseconds]
    set stamp [clock format [clock seconds] -format {%H:%M:%S}]
    if {\$hit eq ""} {
        puts "\$stamp 这次捕获里没有上升沿"
        continue
    }
    lassign \$hit sample previous bus
    set level [expr {\$bus & 1}]
    set pulse [expr {(\$bus >> 1) & 1}]
    set tx1_ok [expr {(\$bus >> 2) & 1}]
    set tx2_ok [expr {(\$bus >> 3) & 1}]
    set tx4_ok [expr {(\$bus >> 4) & 1}]
    set before_level "无"
    if {\$previous ne ""} {
        set before_level [expr {\$previous & 1}]
    }
    incr seen
    set gap ""
    if {\$previous_ms ne ""} {
        set gap [format " 距上一次打印 %d ms" [expr {\$now_ms - \$previous_ms}]]
    }
    set previous_ms \$now_ms
    puts [format "%s 第 %d 个上升沿 样点 %s 电平 %s→%d 脉冲 %d frame_ok tx1=%d tx2=%d tx4=%d%s" \\
        \$stamp \$seen \$sample \$before_level \$level \$pulse \$tx1_ok \$tx2_ok \$tx4_ok \$gap]
}
close_hw_manager
exit 0
EOF

exec "$VIVADO" -mode batch -notrace -source "$TCL"
