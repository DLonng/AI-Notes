#!/usr/bin/env python
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    turtle_name = rospy.get_param('turtle', 'turtle2')
    spawner(4, 2, 0, turtle_name)

    turtle_vel = rospy.Publisher(
        '%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            #trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
            trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time.now(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 4 * \
            math.atan2(trans.transform.translation.y,
                       trans.transform.translation.x)
        msg.linear.x = 0.5 * \
            math.sqrt(trans.transform.translation.x ** 2 +
                      trans.transform.translation.y ** 2)

        turtle_vel.publish(msg)

        rate.sleep()
