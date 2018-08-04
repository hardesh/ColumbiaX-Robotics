#!/usr/bin/env python
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def message_from_transform(T):
    msg = geometry_msgs.msg.Transform()
    q = tf.transformations.quaternion_from_matrix(T)
    translation = tf.transformations.translation_from_matrix(T)

    msg.translation.x = translation[0]
    msg.translation.y = translation[1]
    msg.translation.z = translation[2]
    msg.rotation.x = q[0]
    msg.rotation.y = q[1]
    msg.rotation.z = q[2]
    msg.rotation.w = q[3]
    return msg

def publish_transforms():
    object_transform = geometry_msgs.msg.TransformStamped()
    object_T = tf.transformations.concatenate_matrices(
                tf.transformations.quaternion_matrix(
                tf.transformations.quaternion_from_euler(0.79,0.0,0.79)),
                tf.transformations.translation_matrix([0.0,1.0,1.0])
    )
    object_inverse = tf.transformations.inverse_matrix(object_T)
    object_transform.transform = message_from_transform(object_T)

    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    br.sendTransform(object_transform)

    ################################################

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"

    robot_T = tf.transformations.concatenate_matrices(
            tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_about_axis(1.5, (0,0,1)) ),
            tf.transformations.translation_matrix([0,-1,0])
    )
    robot_inverse = tf.transformations.inverse_matrix(robot_T)
    robot_transform.transform = message_from_transform(robot_T)
    br.sendTransform(robot_transform)

    ##############################################

    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"

    camera_T = tf.transformations.concatenate_matrices(
            tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_from_euler(0,0,0)),
            tf.transformations.translation_matrix([0.0,0.1,0.1])
    )
    camera_inverse = tf.transformations.inverse_matrix(camera_T)
    t1 = tf.transformations.concatenate_matrices(camera_inverse,robot_inverse,object_T)
    dir = tf.transformations.translation_from_matrix(t1)
    x_axis = [1,0,0]
    axis_rot = numpy.cross(x_axis,dir)
    cos_angle = numpy.vdot(x_axis,dir) / numpy.linalg.norm(dir)
    angle = numpy.arccos(cos_angle)
    camera_q = tf.transformations.quaternion_about_axis(angle,axis_rot)
    camera_T = tf.transformations.concatenate_matrices(
            camera_T,tf.transformations.quaternion_matrix(camera_q)
    )
    camera_transform.transform = message_from_transform(camera_T)
    br.sendTransform(camera_transform)

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
