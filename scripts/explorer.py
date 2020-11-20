#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
import tf

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = True


class GoalPoseCommander:

    def __init__(self):
        rospy.init_node('turtlebot_explorer', anonymous=True)
        # initialize variables
        self.x_g = None
        self.y_g = None
        self.theta_g = None
        self.goal_pose_received = False
        self.trans_listener = tf.TransformListener()
        self.start_time = rospy.get_rostime()
        # command pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.goal_idx = 0
        self.goals = [[3.3, 0.25, 3], [2.25, 0.45, 2.355], [0.25, 0.37, 1.5], [0.3, 2.1, 0.7], [2.3, 2.5, -1.5],[3.15,1.6,0.0]]

    def load_next_goal(self):
        try:
            self.x_g, self.y_g, self.theta_g = self.goals[self.goal_idx]
        except:
            print("No more goal, you can exit now")
        self.goal_idx += 1

    def reset_goal(self):
        self.x_g, self.y_g, self.theta_g = None, None, None

    def publish_goal_pose(self):
        """ sends th """
        if self.x_g is not None:
            pose_g_msg = Pose2D()
            pose_g_msg.x = self.x_g
            pose_g_msg.y = self.y_g
            pose_g_msg.theta = self.theta_g
            self.nav_goal_publisher.publish(pose_g_msg)

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t = rospy.get_rostime()
            next = raw_input("Next: ")
            if next == 'y':
                self.load_next_goal()
                self.publish_goal_pose()
            rate.sleep()


if __name__ == '__main__':
    sup = GoalPoseCommander()
    print("Remember to start woof")
    try:
        sup.loop()
    except rospy.ROSInterruptException:
        pass
