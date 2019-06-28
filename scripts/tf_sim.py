#!/usr/bin/env python
import rospy
import tf


if __name__ == '__main__':
    rospy.init_node("tf_mimic")

    source_frame = rospy.get_param('~source_frame', 'world')
    child_frame = rospy.get_param('~child_frame', 'marker')
    sim_child_frame = rospy.get_param('~sim_child_frame', 'sim_marker')

    rate = rospy.Rate(50)

    l = tf.TransformListener(True, rospy.Duration(1.0))
    b = tf.TransformBroadcaster()
    pos = [0, 0, 0]
    orient = [0, 0, 0, 1]


    while not rospy.is_shutdown():
        try:
            (trans,rot) = l.lookupTransform(source_frame, child_frame, rospy.Time(0))
            pos = trans
            orient = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            trans = pos
            rot = orient

        b.sendTransform(pos, orient, rospy.Time.now(), sim_child_frame, source_frame,)


        rate.sleep()
