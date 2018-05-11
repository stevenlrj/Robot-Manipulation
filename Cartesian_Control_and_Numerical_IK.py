#!/usr/bin/env python
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import rospy
import numpy
import tf
import tf2_ros
import geometry_msgs.msg
import angle_axis
from cartesian_control.msg import CartesianCommand
import random


def convert_from_message(t):
    trans = tf.transformations.translation_matrix((t.translation.x,
                                                  t.translation.y,
                                                  t.translation.z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
    T = numpy.dot(trans,rot)
    return T

class CartesianControl(object):
    def __init__(self):
        # Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()
        
        # This is where we hold general info about the joints
        self.num_joints = 0
        self.link = []
        self.joint = []
        self.joint_names = []
        self.joint_axes = []
        self.joint_index = []
        self.cartesian = False
        self.ik_command = False
        

        # Prepare general information about the robot
        self.get_joint_info()
        
        # This is where we'll hold the most recent joint angle information we receive on the topic
        self.current_joint_state = JointState()
        self.secondary_objective = False
        self.q0_target = 0
        self.x_desired = tf.transformations.identity_matrix()
        self.cartesian_command = CartesianCommand()
        self.q_position = []

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        rospy.Subscriber("/cartesian_command", CartesianCommand, self.cartesian_callback)
        rospy.Subscriber("ik_command", geometry_msgs.msg.Transform, self.ik_callback)

    # The actual callback only stores the received information
    def joint_callback(self, joint_state):
        self.current_joint_state = joint_state

    def cartesian_callback(self, msg):
	
        self.cartesian = True
	self.ik_command = False
    	self.cartesian_command = msg
        self.x_desired = convert_from_message(msg.x_target)
        self.secondary_objective = msg.secondary_objective
        self.q0_target = msg.q0_target

    def ik_callback(self, Transform):
        self.ik_command = True
	self.cartesian = False
        self.x_desired = convert_from_message(Transform)

        
    def get_joint_info(self):
        self.num_joints = 0
        self.link = []
        self.joint = []
        self.joint_names = []
        self.joint_axes = []
        link = self.robot.get_root()
        self.link.append(link)
        self.joint_index = []

        while True:
            if link not in self.robot.child_map: break            
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
	    self.joint.append(current_joint)        
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link
            self.link.append(link)
	for i in range(self.num_joints):
		if self.joint_axes[i][0] != 0:
			self.joint_index.append(3)
                elif self.joint_axes[i][1] != 0 :
			self.joint_index.append(4)
                elif self.joint_axes[i][2] != 0:
			self.joint_index.append(5)

	

			



    # Uses the information passed in under joint_state as well as that
    # in self.joint_names and self.joint_axes to comput
    # the list joint_transforms as well as the current end-effector pose x_current.
    # Both Cartesian Control and IK will make use of this.


    def forward_kinematics(self, q_position):
	j = self.joint 
        link = self.link 
        b_ee_transform = []
        joint_ee_transform = []

    	Tl11 = tf.transformations.euler_matrix(j[0].origin.rpy[0], j[0].origin.rpy[1], j[0].origin.rpy[2], 'sxyz') 
    	Tl12 = tf.transformations.translation_matrix(j[0].origin.xyz)
    
    	Tl21 = tf.transformations.euler_matrix(j[1].origin.rpy[0], j[1].origin.rpy[1], j[1].origin.rpy[2], 'sxyz')
    	Tl22 = tf.transformations.translation_matrix(j[1].origin.xyz)
      
    	Tl31 = tf.transformations.euler_matrix(j[2].origin.rpy[0], j[2].origin.rpy[1], j[2].origin.rpy[2], 'sxyz')
    	Tl32 = tf.transformations.translation_matrix(j[2].origin.xyz)
 

    	Tl41 = tf.transformations.euler_matrix(j[3].origin.rpy[0], j[3].origin.rpy[1], j[3].origin.rpy[2], 'sxyz')
    	Tl42 = tf.transformations.translation_matrix(j[3].origin.xyz)

    	Tl51 = tf.transformations.euler_matrix(j[4].origin.rpy[0], j[4].origin.rpy[1], j[4].origin.rpy[2], 'sxyz')
    	Tl52 = tf.transformations.translation_matrix(j[4].origin.xyz)
   
    	Tl61 = tf.transformations.euler_matrix(j[5].origin.rpy[0], j[5].origin.rpy[1], j[5].origin.rpy[2], 'sxyz')
    	Tl62 = tf.transformations.translation_matrix(j[5].origin.xyz)
   
    	Tl71 = tf.transformations.euler_matrix(j[6].origin.rpy[0], j[6].origin.rpy[1], j[6].origin.rpy[2], 'sxyz')
    	Tl72 = tf.transformations.translation_matrix(j[6].origin.xyz)
  
    	Tl81 = tf.transformations.euler_matrix(j[7].origin.rpy[0], j[7].origin.rpy[1], j[7].origin.rpy[2], 'sxyz')
    	Tl82 = tf.transformations.translation_matrix(j[7].origin.xyz)
        
    	Tj = []

	q_index = 0

    	for i in range(0, 8):
        	if j[i].type == "revolute":
               	 	T = tf.transformations.rotation_matrix(q_position[q_index], j[i].axis)
               		Tj.append(T)
			q_index += 1
			
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

        T1 = tf.transformations.concatenate_matrices(TF1, Tl22, Tl21)
        T1v = numpy.linalg.inv(T1)  
        T1_ee = numpy.dot(T1v, TF8)
        b_ee_transform.append(T1)
	joint_ee_transform.append(T1_ee)

    	T2 = tf.transformations.concatenate_matrices(T1, Tj[1], Tl32, Tl31)
        T2v = numpy.linalg.inv(T2)
	T2_ee = numpy.dot(T2v, TF8)
        b_ee_transform.append(T2)
	joint_ee_transform.append(T2_ee)
 
    	T3 = tf.transformations.concatenate_matrices(T2, Tj[2], Tl42, Tl41)
        T3v = numpy.linalg.inv(T3)
        b_ee_transform.append(T3)
	T3_ee = numpy.dot(T3v, TF8)
	joint_ee_transform.append(T3_ee)

    	T4 = tf.transformations.concatenate_matrices(T3, Tj[3], Tl52, Tl51)
	T4v = numpy.linalg.inv(T4)
	T4_ee = numpy.dot(T4v, TF8)
	b_ee_transform.append(T4)
	joint_ee_transform.append(T4_ee)
	
    	T5 = tf.transformations.concatenate_matrices(T4, Tj[4], Tl62, Tl61)
	T5v = numpy.linalg.inv(T5)
	T5_ee = numpy.dot(T5v, TF8)	
	b_ee_transform.append(T5) 
	joint_ee_transform.append(T5_ee)
	
        T6 = tf.transformations.concatenate_matrices(T5, Tj[5], Tl72, Tl71)
	T6v = numpy.linalg.inv(T6)
	T6_ee = numpy.dot(T6v, TF8)
	b_ee_transform.append(T6)
	joint_ee_transform.append(T6_ee)
 
        T7 = tf.transformations.concatenate_matrices(T6, Tj[6], Tl82, Tl81)
	T7v = numpy.linalg.inv(T7)
	T7_ee = numpy.dot(T7v, TF8)
	b_ee_transform.append(T7)
	joint_ee_transform.append(T7_ee)
	
        x_current = TF8
        base_joint_transforms = b_ee_transform

	self.cartesian = False
        
        return (base_joint_transforms, joint_ee_transform, x_current)

    def q_position_js(self, joint_state):
	JS = joint_state
	j = self.joint 
        link = self.link 
	q_position = []

	for i in range(0, 8):
        	if j[i].type == "revolute":
        		a = JS.name.index(j[i].name)
			q_position.append(JS.position[a])
	self.q_position = q_position
	return(q_position)

    def ik_iteration(self, x_desired):
	joint_index = self.joint_index
	num_joints = self.num_joints
	q_position = numpy.zeros(num_joints)
	max_step = 1000
	
	for i in range(num_joints):
		rand_q = random.uniform(0.0, 2.0)
		q_position[i] = rand_q
	
 	for i in range(max_step + 1):
		base_joint_transforms, joint_ee_transform, x_current = self.forward_kinematics(q_position)	
                j, j_pinv, js = cartesian_control(joint_ee_transform, num_joints, joint_index)
		P = 0.6	
		v_r = numpy.dot(numpy.linalg.inv(x_current),x_desired)
		v_angle, v_axis = angle_axis.rotation_from_matrix(v_r)
		v_ee = numpy.zeros(6)
		v_ee[0:3] = tf.transformations.translation_from_matrix(v_r)
		v_ee[3:6] = numpy.dot(v_angle, v_axis)
		v_ee = P * (v_ee)
		delta_q = numpy.dot(j_pinv, v_ee)
		q_position += delta_q
	print(q_position)
        self.ik_command = False
	return(q_position)

    def secondary(self, j, j_pinv, js, x_desired, x_current, secondary_objective, q0_target, q_position, num_joint):
        P = 0.6	
	v_transform = numpy.dot(numpy.linalg.inv(x_current), x_desired)
	v_angle, v_axis = angle_axis.rotation_from_matrix(v_transform[:3, :3])
	v_ee = numpy.zeros(6)
	v_ee[0:3] = tf.transformations.translation_from_matrix(v_transform)
	v_ee[3:6] = numpy.dot(v_angle, v_axis)
	v_ee = P * (v_ee)
	
        if secondary_objective == True:
		q_sec = numpy.zeros(num_joint)
                q_sec[0] = P * (q0_target - q_position[0])
                I = numpy.identity(num_joint)
                q_null = numpy.dot((I - numpy.dot(j_pinv, j)), q_sec)
                q_dot = numpy.dot(js, v_ee) + q_null
		self.secondary_objective = False
		
	else:
		q_dot = numpy.dot(js, v_ee)
	
	return(q_dot)

def s_function(translation_transform):
        x = translation_transform[0]
        y = translation_transform[1]
        z = translation_transform[2]
	s_translation_transform = numpy.array([[0, -z, y],[z, 0, -x],[-y, x, 0]])
	return(s_translation_transform)
	
def cartesian_control(joint_ee_transform, num_joint, joint_index):
	jacobian_transform = numpy.zeros((6, num_joint))
        for i in range(num_joint):
                ee_joint_transform = numpy.linalg.inv(joint_ee_transform[i])
		rotation_transform = ee_joint_transform[0:3, 0:3] 
                translation_transform = joint_ee_transform[i][0:3, 3]
                s_transform = numpy.dot(-rotation_transform, s_function(translation_transform))
                v_transform = numpy.zeros((6, 6))
                v_transform[0:3, 0:3] = rotation_transform
                v_transform[0:3, 3:6] = s_transform
                v_transform[3:6, 3:6] = rotation_transform
                jacobian_transform[:, i] = v_transform[:,joint_index[i]]
	if num_joint > 6:
		jacobian_transform[:, 3] = - jacobian_transform[:, 3]
        j = jacobian_transform
        j_pinv = numpy.linalg.pinv(jacobian_transform)
        js =numpy.linalg.pinv(jacobian_transform, 1.0e-2)
        return(j, j_pinv, js)



if __name__ == '__main__':
        rospy.init_node('ccik_starter', anonymous=True)
        pub_cc = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        pub_ik = rospy.Publisher('/joint_command', JointState, queue_size=10)
        model = CartesianControl()
        rospy.sleep(0.1)
        
	while not rospy.is_shutdown():
            JS = model.current_joint_state
            num_joint = model.num_joints
            joint_index = model.joint_index
            joint_axes = model.joint_axes
            joint_names = model.joint_names
	    q_position = model.q_position_js(JS)
	    	
            cartesian = model.cartesian
            secondary_objective = model.secondary_objective
            q0_target = model.q0_target
            ik_command = model.ik_command	    
	    
            if cartesian == True:
		
		base_joint_transforms, joint_ee_transform, x_current = model.forward_kinematics(q_position)	
                j, j_pinv, js = cartesian_control(joint_ee_transform, num_joint, joint_index)
	    	x_desired = model.x_desired
		
            	q_dot = model.secondary(j, j_pinv, js, x_desired, x_current, secondary_objective, q0_target, q_position, num_joint)

            	joint_velocity = JointState()
            	joint_velocity.name = joint_names
            	joint_velocity.velocity = q_dot
            	pub_cc.publish(joint_velocity)
                

            elif ik_command == True:
		
		x_desired = model.x_desired
		
		q_p = model.ik_iteration(x_desired)
 		
                joint_command = JointState()
                joint_command.name = joint_names
		joint_command.position = q_p
                pub_ik.publish(joint_command)
                
            
            rospy.sleep(0.1)

    
