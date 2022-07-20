

import rosbag

with rosbag.Bag('iDog_modified.bag','w') as outbag:
    for topic, msg, t in rosbag.Bag('/home/idac/catkin_ws_project/iDog.bag').read_messages():
        print(t)

        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)