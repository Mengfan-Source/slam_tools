#!/bin/bash
BAG=/home/xmf/xmf_bags/S20/20260305_xuhui_and_diku/all_2026-03-05-08-28-01_seg1.bag
WINDOW=600                                      # 每段 30 秒

# 读取浮点时间
START_T=$(rosbag info -k start -y "$BAG")
END_T=$(rosbag info -k end -y "$BAG")

# 计算总段数（向上取整）
SEGS=$(echo "scale=0; ($END_T - $START_T) / $WINDOW + 1" | bc)

for ((i=0; i<SEGS; i++)); do
  T0=$(echo "$START_T + $i * $WINDOW" | bc -l)
  T1=$(echo "$T0 + $WINDOW" | bc -l)
  rosbag filter "$BAG" "${BAG%.bag}_seg$i.bag" \
    "t.to_sec() >= $T0 and t.to_sec() < $T1"
done
