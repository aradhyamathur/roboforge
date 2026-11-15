#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('talker')
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)
    count = 0
    while not rospy.is_shutdown():
        pub.publish(String(data=f'hello #{count}'))
        count += 1
        rate.sleep()


if __name__ == '__main__':
    main()
