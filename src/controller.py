#!/usr/bin/env python

import copy
import math
from visualization_msgs.msg import *

import rospy
import tf2_ros
# from tf.transformations import *
import tf2_geometry_msgs
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
from docking.msg import *

header_ = Header()
header_.frame_id = "laser"

ROBOT_X_BASIS_VEC = Vector3(1, 0, 0)
ROBOT_X_BASIS_VECSTMP = Vector3Stamped(header_,ROBOT_X_BASIS_VEC)
DOCK_X_BASIS_VEC = Vector3(1, 0, 0)
DOCK_X_BASIS_VECSTMP = Vector3Stamped(header_, DOCK_X_BASIS_VEC)
DEBUG = True
global LOS
# LOS = docking.msg.LineOfSight()


def dockPoseCallback(dockPose):
    global LOS
    LOS = docking.msg.LineOfSight()

    global tfBuffer
    dockTF = tfBuffer.lookup_transform('laser', 'dock_truth', rospy.Time())
    LOS.header = dockTF.header
    getLineOfSightFromTF(dockTF)

    # rospy.loginfo("RECEIVED DOCK POSE")
    # rospy.loginfo(dockPose)
    # getLineOfSightFromPose(dockPose)


def getLineOfSightFromTF(dockTF):
    global LOS
    LOS.endPoint = Point(dockTF.transform.translation.x, dockTF.transform.translation.y, 0)
    LOS.length = math.sqrt(LOS.endPoint.x*LOS.endPoint.x + LOS.endPoint.y*LOS.endPoint.y)
    LOS.slope = LOS.endPoint.y / LOS.endPoint.x
    LOS.arrow = genLOSArrow()


    global ROBOT_X_BASIS_VEC
    rospy.loginfo("ROBOT_X_BASIS_VEC %s", ROBOT_X_BASIS_VECSTMP)
    LOS.delta = getAngleBetweenVectors(ROBOT_X_BASIS_VEC,LOS.endPoint)
    rospy.loginfo("DELTA ANGLE BTWN ROBOT_X & LOS %s", LOS.delta)

    global DOCK_X_BASIS_VECSTMP
    rospy.loginfo("DOCK_X_BASIS_VECSTMP %s", DOCK_X_BASIS_VECSTMP)
    DOCK_X_BASIS_VECSTMP = tf2_geometry_msgs.do_transform_vector3(ROBOT_X_BASIS_VECSTMP, dockTF)
    LOS.phi = getAngleBetweenVectors(LOS.endPoint,DOCK_X_BASIS_VEC)
    rospy.loginfo("PHI ANGLE BTWN LOS & DOCK_X %s", LOS.phi)


# def getLineOfSightFromPose(dockPose):
#     LOS.endPoint = Point(dockPose.pose.position.x, dockPose.pose.position.y, 0)
#     LOS.length = math.sqrt(LOS.endPoint.x * LOS.endPoint.x + LOS.endPoint.y * LOS.endPoint.y)
#     LOS.slope = LOS.endPoint.y / LOS.endPoint.x
#     LOS.arrow = genLOSArrow()

#     tempDockTF = TransformStamped()
#     tempDockTF.header.frame_id = "laser"
#     tempDockTF.child_frame_id = "dock"
#     tempDockTF.transform.translation = dockPose.pose.position
#     tempDockTF.transform.rotation = dockPose.pose.orientation

#     LOS.delta = getAngleBetweenVectors(ROBOT_X_BASIS_VEC, LOS.endPoint)
#     rospy.loginfo("DELTA ANGLE BTWN ROBOT_X & LOS %s", LOS.delta)

#     DOCK_X_BASIS_VECSTMP = tf2_geometry_msgs.do_transform_vector3(ROBOT_X_BASIS_VECSTMP, tempDockTF)
#     LOS.phi = getAngleBetweenVectors(LOS.endPoint, DOCK_X_BASIS_VEC)
#     rospy.loginfo("PHI ANGLE BTWN LOS & DOCK_X %s", LOS.phi)


def genLOSArrow():
    global LOS
    arrow = Marker()
    arrow.header.frame_id = LOS.header.frame_id
    rospy.loginfo("LOS.header.frame_id %s", LOS.header.frame_id)
    rospy.loginfo("arrow.header.frame_id %s", arrow.header.frame_id)
    arrow.header.stamp = rospy.Time.now()
    # arrow.ns = ""
    arrow.action = arrow.ADD
    arrow.type = arrow.ARROW
    arrow.id = 0

    arrow.scale.x = 0.1
    arrow.scale.y = 0.2
    arrow.scale.z = 0.2

    arrow.color.a = 1.0
    arrow.color.r = 0.5
    arrow.color.g = 0.1
    arrow.color.b = 0.1

    arrow.points = [Point(0, 0, 0), Point(LOS.endPoint.x, LOS.endPoint.x, 0)]

    return arrow


def getAngleBetweenVectors(v0, v1):
    # tf2::tf2Angle (const Vector3 &v1, const Vector3 &v2)
    rospy.loginfo("V0")
    rospy.loginfo(v0)
    rospy.loginfo("V1")
    rospy.loginfo(v1)
    dot = (v0.x * v1.x) + (v0.y * v1.y)
    magV0 = math.sqrt(v0.x * v0.x + v0.y * v0.y)
    magV1 = math.sqrt(v1.x * v1.x + v1.y * v1.y)
    magVectors = magV0 * magV1
    return math.acos(dot / magVectors)


def main():
    rospy.init_node("controller")

    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    dock_pose_sub = rospy.Subscriber('dock_pose', PoseStamped, dockPoseCallback)
    LOS_marker_pub = rospy.Publisher('LOS', Marker, queue_size=10)
    r = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        global LOS
        LOS_marker_pub.publish(LOS.arrow)
        r.sleep()


if __name__ == '__main__':
    main()