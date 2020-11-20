#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from asl_turtlebot.msg import DetectedObject, DetectedObjectList

bot_pose_x,bot_pose_y = 0.0 , 0.0
bot_pose_q1,bot_pose_q2,bot_pose_q3,bot_pose_q4 = 0.0 , 0.0 , 0.0 , 0.0
obj_name = "Nothing"

def pose_callback(data):
    global bot_pose_x,bot_pose_y,bot_pose_q1,bot_pose_q2,bot_pose_q3,bot_pose_q4

    bot_pose_x = data.pose.pose.position.x
    bot_pose_y = data.pose.pose.position.y
    bot_pose_q1 = data.pose.pose.orientation.x
    bot_pose_q2 = data.pose.pose.orientation.y
    bot_pose_q3 = data.pose.pose.orientation.z
    bot_pose_q4 = data.pose.pose.orientation.w

def getfovpoints(dist=1,hght=0.3):
    # TODO: calculate the actual wireframe size
    square_frame = np.array([[0,-0.5,0],[0,0.5 ,0],
                             [0,0.5 ,0],[0,0.5 ,1],
                             [0,0.5 ,1],[0,-0.5,1],
                             [0,-0.5,1],[0,-0.5,0]]) + np.array([0,0,-0.5+hght])
    # square_frame[square_frame[:,2]<0,2] = 0
    pyramid = np.array([[-dist,0,hght],square_frame[0],
                        [-dist,0,hght],square_frame[2],
                        [-dist,0,hght],square_frame[4],
                        [-dist,0,hght],square_frame[6]])
    pts = np.concatenate((square_frame,pyramid))
    
    fovpoints = []
    for i in range(pts.shape[0]):
        tmp_pt = Point()
        tmp_pt.x, tmp_pt.y, tmp_pt.z = pts[i]
        fovpoints.append(tmp_pt)
    return fovpoints

def detected_objects_name_callback(msg):
    global obj_name
    rospy.loginfo("There are %i detected objects" % len(msg.objects))
    obj_name = msg.objects[0]
    # #self.detected_objects = msg
    # if msg.objects[0] == 'cat':
    #     marker.type = Marker.CUBE #
    # elif msg.objects[0] == 'dog':
    #     marker.type = Marker.SPHERE #
    # else:
    #     marker.type = Marker.LINE_LIST #
    

def publisher():
    global bot_pose_x,bot_pose_y,bot_pose_q1,bot_pose_q2,bot_pose_q3,bot_pose_q4, obj_name

    vis_pub = rospy.Publisher('fov_markers', Marker, queue_size=10)
    obj_pub = rospy.Publisher('obj_markers', Marker, queue_size=10)
    ani_pub = rospy.Publisher('ani_markers', Marker, queue_size=10)

    rospy.Subscriber('odom',Odometry,pose_callback)

    rospy.Subscriber('/detector/objects', DetectedObjectList, detected_objects_name_callback, queue_size=10)

    rospy.init_node('turtlebot_fov', anonymous=True)
    rate = rospy.Rate(30)

    frame_dist = 0.5
    while not rospy.is_shutdown():
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        marker.id = 0

        marker.type = Marker.LINE_LIST # Line list

        bot_th = np.arctan2(2.0*(bot_pose_q4*bot_pose_q3+bot_pose_q1*bot_pose_q2) , 1.0-2.0*(bot_pose_q2**2+bot_pose_q3**2))
        marker.pose.position.x = bot_pose_x + frame_dist*np.cos(bot_th)
        marker.pose.position.y = bot_pose_y + frame_dist*np.sin(bot_th)
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = bot_pose_q1
        marker.pose.orientation.y = bot_pose_q2
        marker.pose.orientation.z = bot_pose_q3
        marker.pose.orientation.w = bot_pose_q4

        marker.scale.x = 0.01

        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.3
        marker.color.g = 0.711
        marker.color.b = 0.672

        fovpoints = getfovpoints(dist=frame_dist)
        marker.points = fovpoints
        vis_pub.publish(marker)      

        # ============== Detected Object ===================
        obj_marker = Marker()

        obj_marker.header.frame_id = "map"
        obj_marker.header.stamp = rospy.Time()

        obj_marker.id = 1

        obj_marker.type = Marker.TEXT_VIEW_FACING # Line list

        obj_marker.pose.position.x = bot_pose_x + 1.1*frame_dist*np.cos(bot_th)
        obj_marker.pose.position.y = bot_pose_y + 1.1*frame_dist*np.sin(bot_th)
        obj_marker.pose.position.z = 0.9

        obj_marker.pose.orientation = marker.pose.orientation

        obj_marker.text = "Detected Object: " + obj_name
        obj_marker.scale.z = 0.1

        obj_marker.color = marker.color
        
        obj_pub.publish(obj_marker)    

        # ============== Dog/Cat ===================
        ani_marker = Marker()

        ani_marker.header.frame_id = "map"
        ani_marker.header.stamp = rospy.Time()

        ani_marker.id = 2

        ani_marker.type = Marker.TEXT_VIEW_FACING # Line list

        ani_marker.pose.position.x = bot_pose_x
        ani_marker.pose.position.y = bot_pose_y
        ani_marker.pose.position.z = 0.9

        ani_marker.pose.orientation = marker.pose.orientation
        if obj_name == "dog":
            ani_marker.text = "Woof!"
        elif obj_name == "cat":
            ani_marker.text = "Meow!"
        else:
            ani_marker.text = ""

        ani_marker.scale.z = 1

        ani_marker.color = marker.color
        
        ani_pub.publish(ani_marker)    
  
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
