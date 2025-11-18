#!/usr/bin/env python
import rosbag
from sensor_msgs.msg import PointCloud2

bag = rosbag.Bag('/home/xmf/xmf_bags/fromX30/rtk/2025-11-18-10-33-39.bag')
_, msg, _ = next(bag.read_messages(topics=['/lidar_points']))  # 取第一条
for f in msg.fields:
    print('{:12} offset={:2}  datatype={}  count={}'.format(f.name, f.offset, f.datatype, f.count))
print('point_step =', msg.point_step)   # 26 字节，与你给出的一致
bag.close()
