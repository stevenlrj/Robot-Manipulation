#!/usr/bin/env python
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import rospy
import numpy
import tf
import tf2_ros
import geometry_msgs.msg

def message_from_transform(T, char1, char2):   
    a = geometry_msgs.msg.TransformStamped()
    q = tf.transformations.quaternion_from_matrix(T)
    translation = tf.transformations.translation_from_matrix(T)
    a.header.stamp = rospy.Time.now()
    a.header.frame_id = char1
    a.child_frame_id = char2 
    a.transform.translation.x = translation[0]
    a.transform.translation.y = translation[1]
    a.transform.translation.z = translation[2]
    a.transform.rotation.x = q[0]
    a.transform.rotation.y = q[1]
    a.transform.rotation.z = q[2]
    a.transform.rotation.w = q[3]
    return a

def callback(JointState):
    
    JS = JointState
    
    robot = URDF.from_parameter_server()
    br = tf2_ros.TransformBroadcaster()
    link = robot.get_root()

    (joint1, link1) = robot.child_map[link][0]
    (joint2, link2) = robot.child_map[link1][0]
    (joint3, link3) = robot.child_map[link2][0]
    (joint4, link4) = robot.child_map[link3][0]    
    (joint5, link5) = robot.child_map[link4][0]
    (joint6, link6) = robot.child_map[link5][0]
    (joint7, link7) = robot.child_map[link6][0]
    (joint8, link8) = robot.child_map[link7][0]
   


    j = []  
     
    j1 = robot.joint_map[joint1]
    j.append(j1)
    Tl11 = tf.transformations.euler_matrix(j1.origin.rpy[0], j1.origin.rpy[1], j1.origin.rpy[2], 'sxyz') 
    Tl12 = tf.transformations.translation_matrix(j1.origin.xyz)
    
    j2 = robot.joint_map[joint2]
    j.append(j2)
    Tl21 = tf.transformations.euler_matrix(j2.origin.rpy[0], j2.origin.rpy[1], j2.origin.rpy[2], 'sxyz')
    Tl22 = tf.transformations.translation_matrix(j2.origin.xyz)
      
    j3 = robot.joint_map[joint3]
    j.append(j3)
    Tl31 = tf.transformations.euler_matrix(j3.origin.rpy[0], j3.origin.rpy[1], j3.origin.rpy[2], 'sxyz')
    Tl32 = tf.transformations.translation_matrix(j3.origin.xyz)
 
    j4 = robot.joint_map[joint4]
    j.append(j4)
    Tl41 = tf.transformations.euler_matrix(j4.origin.rpy[0], j4.origin.rpy[1], j4.origin.rpy[2], 'sxyz')
    Tl42 = tf.transformations.translation_matrix(j4.origin.xyz)

    j5 = robot.joint_map[joint5]
    j.append(j5)
    Tl51 = tf.transformations.euler_matrix(j5.origin.rpy[0], j5.origin.rpy[1], j5.origin.rpy[2], 'sxyz')
    Tl52 = tf.transformations.translation_matrix(j5.origin.xyz)
   
    j6 = robot.joint_map[joint6]
    j.append(j6)
    Tl61 = tf.transformations.euler_matrix(j6.origin.rpy[0], j6.origin.rpy[1], j6.origin.rpy[2], 'sxyz')
    Tl62 = tf.transformations.translation_matrix(j6.origin.xyz)
   
    j7 = robot.joint_map[joint7]
    j.append(j7)
    Tl71 = tf.transformations.euler_matrix(j7.origin.rpy[0], j7.origin.rpy[1], j7.origin.rpy[2], 'sxyz')
    Tl72 = tf.transformations.translation_matrix(j7.origin.xyz)
  
    
    j8 = robot.joint_map[joint8]
    j.append(j8)
    Tl81 = tf.transformations.euler_matrix(j8.origin.rpy[0], j8.origin.rpy[1], j8.origin.rpy[2], 'sxyz')
    Tl82 = tf.transformations.translation_matrix(j8.origin.xyz)

    
    Tj = []

    for i in range(0, 8):
        if j[i].type == "revolute":
                a = JS.name.index(j[i].name)
                T = tf.transformations.rotation_matrix(JS.position[a], j[i].axis)
                Tj.append(T)
        else:  
                T = tf.transformations.identity_matrix()
                Tj.append(T)

    TF0 = tf.transformations.identity_matrix()
    TF1 = tf.transformations.concatenate_matrices(TF0, Tl12, Tl11, Tj[0])
    TF2 = tf.transformations.concatenate_matrices(TF1, Tl22, Tl21, Tj[1])
    TF3 = tf.transformations.concatenate_matrices(TF2, Tl32, Tl31, Tj[2]) 
    TF4 = tf.transformations.concatenate_matrices(TF3, Tl42, Tl41, Tj[3])
    TF5 = tf.transformations.concatenate_matrices(TF4, Tl52, Tl51, Tj[4])
    TF6 = tf.transformations.concatenate_matrices(TF5, Tl62, Tl61, Tj[5]) 
    TF7 = tf.transformations.concatenate_matrices(TF6, Tl72, Tl71, Tj[6])
    TF8 = tf.transformations.concatenate_matrices(TF7, Tl82, Tl81, Tj[7]) 
   
  
    T1_stamped = message_from_transform(TF1, link, link1)
    br.sendTransform(T1_stamped)
   
    T2_stamped = message_from_transform(TF2, link, link2 )
    br.sendTransform(T2_stamped)
  
    T3_stamped = message_from_transform(TF3, link, link3 )
    br.sendTransform(T3_stamped)

    T4_stamped = message_from_transform(TF4, link, link4 )
    br.sendTransform(T4_stamped)

    T5_stamped = message_from_transform(TF5, link, link5)
    br.sendTransform(T5_stamped)

    T6_stamped = message_from_transform(TF6, link, link6)
    br.sendTransform(T6_stamped)
    
    T7_stamped = message_from_transform(TF7, link, link7)
    br.sendTransform(T7_stamped)

    T8_stamped = message_from_transform(TF8, link, link8 )
    br.sendTransform(T8_stamped) 
 
def forward_kinematics(): 
        rospy.Subscriber("/joint_states", JointState, callback) 

if __name__ == '__main__':
        rospy.init_node('forward_kinematics', anonymous=True)
        forward_kinematics()
        rospy.spin()

