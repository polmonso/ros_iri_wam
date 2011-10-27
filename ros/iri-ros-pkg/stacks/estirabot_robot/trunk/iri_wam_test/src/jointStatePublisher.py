#!/usr/bin/env python
import roslib; roslib.load_manifest('iri_wam_test')
import rospy
from sensor_msgs.msg import JointState
def jointStatePublisher():
    rospy.init_node('jointStatePublisher')
    name = rospy.get_name()+'/joint_states'
    print name
    pub = rospy.Publisher('joint_states', JointState)
    jointsmsg = JointState()
    while not rospy.is_shutdown():
        jointsmsg.header.stamp = rospy.Time.now()
        jointsmsg.header.frame_id = "wam0"
        for i in range(0, 7):
            jointsmsg.name.append("wamframe")
            jointsmsg.position.append(0.1)
        pub.publish(jointsmsg)
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        jointStatePublisher()
    except rospy.ROSInterruptException: pass


