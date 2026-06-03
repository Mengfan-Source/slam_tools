#!/usr/bin/env python3
import sys
import csv
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import builtin_interfaces.msg

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 extract_timestamps.py <bag_dir> [output.csv]")
        sys.exit(1)

    bag_dir = Path(sys.argv[1])
    output_csv = sys.argv[2] if len(sys.argv) > 2 else "timestamps.csv"

    # 设置 bag 读取选项
    storage_options = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # 获取话题类型映射
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp", "topic"])   # 表头

        while reader.has_next():
            topic_name, serialized_msg, timestamp = reader.read_next()
            # timestamp 是纳秒（int）
            unix_sec = timestamp / 1e9
            writer.writerow([unix_sec, topic_name])

    print(f"已保存到 {output_csv}")

if __name__ == "__main__":
    main()
