#!/usr/bin/env python
from sensor_msgs.msg import JointState
import numpy
import sys
from urdf_parser_py.urdf import URDF
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import random
import angle_axis


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

def convert_to_message(T):
	t = geometry_msgs.msg.Pose()
	position = tf.transformations.translation_from_matrix(T)
	orientation = tf.transformations.quaternion_from_matrix(T)
	t.position.x = position[0]
	t.position.y = position[1]
	t.position.z = position[2]
	t.orientation.x = orientation[0]
	t.orientation.y = orientation[1]
	t.orientation.z = orientation[2]
	t.orientation.w = orientation[3]        
 	return t

def convert_to_matrix(a):
   	t = tf.transformations.translation_matrix((a.translation.x, a.translation.y, a.translation.z))
 	r = tf.transformations.quaternion_matrix((a.rotation.x, a.rotation.y, a.rotation.z, a.rotation.w))
 	T = numpy.dot(t,r)
	return(T)

class MoveArm(object):

    def __init__(self):
	self.robot = URDF.from_parameter_server()
        
        # This is where we hold general info about the joints
        self.num_joints = 0
        self.link = []
        self.joint = []
        self.joint_names = []
        self.joint_axes = []

        # Prepare general information about the robot
        self.get_joint_info()
        self.current_joint_state = JointState()

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
	if self.num_joints == 7:
        	self.group_name = "lwr_arm"
	elif self.num_joints == 6:
		self.group_name = "manipulator"

	self.pub_trajectory = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
	rospy.Subscriber("/joint_states", JointState, self.joint_callback)
	rospy.Subscriber("/motion_planning_goal", geometry_msgs.msg.Transform, self.motion_planning_callback)


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
	if self.num_joints == 6:
				name_1 = self.joint_names[0]
				name_3 = self.joint_names[2]
				self.joint_names[0] = name_3
				self.joint_names[2] = name_1

    def joint_callback(self, joint_state):
        self.current_joint_state = joint_state

    
    def motion_planning_callback(self, Transform):
	matrix_T = convert_to_matrix(Transform)
	q_target = numpy.array(self.ik_iteration(matrix_T))
	valid_target = self.is_state_valid(q_target)
	q_current = numpy.array(self.q_position_js())
	print("Current")
	print(q_current)
	print("Target")
	print(q_target)
	if (len(q_target)) == 0:
		print("IK failed!")
	else:
		if valid_target == True:
			print("IK succeed!")
			if self.num_joints == 6:
				q_1 = q_target[0]
				q_3 = q_target[2]
				q_target[0] = q_3
				q_target[2] = q_1
			path = self.trajectory(q_current, q_target)
			joint_path = self.trajectory_convert(path)
			self.pub_trajectory.publish(joint_path)
		else:
			print("Target is not valid!")
	

    def trajectory_convert(self, path):
	trajectory = JointTrajectory()
	trajectory.joint_names = self.joint_names
	for i in range(len(path)):
		path_point = JointTrajectoryPoint()
		path_point.positions = path[i]
		trajectory.points.append(path_point)
	return(trajectory)

    def random_q(self):
	num = self.num_joints
	q_random = numpy.zeros(num)
	for i in range(num):
		if num == 7:
			q_random[i] = random.uniform(-numpy.pi, numpy.pi)
		elif num == 6:
			q_random[i] = random.uniform(-2*numpy.pi, 2*numpy.pi)
	return(q_random)

		
    def trajectory(self, q_current, q_target):
	q_tree = []
	path= []
	q_index = []
	m = -1
	q_index.append(-1)
	max_step = 5000
	q_tree.append(q_current)
	

	for i in range(max_step):
		q_r = self.random_q()
		q_local, index = self.min_distance(q_tree, q_index, q_r)
		delta_q = (q_r - q_local) / numpy.linalg.norm(q_r - q_local)
		q_new = q_local + delta_q
		valid, q_val = self.validation(q_local, q_new)
		print(i)
		print(valid)
		if valid == True:
			q_tree.append(q_val)
			q_index.append(index)
		valid_target, q_fail = self.validation(q_val, q_target)
		if valid_target == True:
			q_index.append(len(q_tree)-1)
			q_tree.append(q_target)
			print("Success!")
			break
			
		if i == max_step - 1:
			print("Fail, reach max step 5000. Mayber try again!")

						

	for i in range(len(q_tree)):
		path.append(q_tree[m])
		m = q_index[m]
		if m < 0:
			break
	path = path[::-1]
	shortcut_path = self.shortcut(path)
	final_path = self.add_points(shortcut_path)
	return(final_path)

    def add_points(self, path):
	f_path = []
	for i in range(len(path)-1):
		d = path[i+1] - path[i]
		n = numpy.linalg.norm(d)
		if self.num_joints == 7:
			unit = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
		elif self.num_joints == 6:
			unit = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
		num_max = max(numpy.true_divide(abs(d), unit))
		num = numpy.int(numpy.ceil(num_max))
		delta = d / num
		for j in range(num):
			point = path[i] + j * delta
			f_path.append(point)
	f_path.append(path[-1])		
	return(f_path)

		
    def shortcut(self, path):
	shortcut_path = []
	shortcut_path.append(path[0])
	m = 0
	for i in range(1, len(path)-1):
		v, q = self.validation(shortcut_path[m], path[i+1])
		if v == False:
			shortcut_path.append(path[i])
			m += 1	
	shortcut_path.append(path[-1])
	
	return(shortcut_path)	
	
    def validation(self, q_local, q_new):
	d = q_new - q_local
	if self.num_joints == 7:
		unit = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
	elif self.num_joints == 6:
		unit = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
	num_max = max(numpy.true_divide(abs(d), unit))
	num = numpy.int(numpy.ceil(num_max))
	dq = d / num	
	valid = True
	for i in range(1, num+1):
		q_val = q_local + i * dq 
		if self.is_state_valid(q_val) == False:
						q_val = q_val - dq
						valid = False
						break
	return(valid, q_val)
		

    def min_distance(self, q_tree, q_index, q_r):
	num_point = len(q_tree)
	d0 = numpy.inf
	for i in range(num_point):
		d = numpy.linalg.norm(q_tree[i] - q_r)
		if d < d0:
			d0 = d
			q_local = q_tree[i]
			index = i

	return(q_local, index)
	
			
	
        
    def q_position_js(self):
	JS = self.current_joint_state
	j = self.joint  
	q_position = []

	for i in range(0, 8):
        	if j[i].type == "revolute":
        		a = JS.name.index(j[i].name)
			q_position.append(JS.position[a])
	if self.num_joints == 6:
		q_1 = q_position[0]
		q_3 = q_position[2]
		q_position[0] = q_3
		q_position[2] = q_1
	self.q_position = q_position
	return(q_position)

    """ This function will perform IK for a given transform T of the end-effector. It 
    returns a list q[] of 7 values, which are the result positions for the 7 joints of 
    the KUKA arm, ordered from proximal to distal. If no IK solution is found, it 
    returns an empy list.
    """

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

	
        
        return (base_joint_transforms, joint_ee_transform, x_current)

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
		n_q = numpy.linalg.norm(delta_q)
		if n_q < 0.001:
			break
		q_position += delta_q
	q_position = self.normal(q_position)
	if num_joints == 7:
		threshold = 0.1
	elif num_joints == 6:
		threshold = 0.5 
	if n_q > threshold:
		q_position = []
	valid_target = self.is_state_valid(q_position)
	if valid_target == False:
		q_position = []
	return(q_position)

    def normal(self,q):
	for i in range(len(q)):
		if abs(q[i]) > numpy.pi:
			if q[i] < 0:
				q[i] = -(abs(q[i])%(2*numpy.pi))
			else:
				q[i] = q[i]%(2*numpy.pi)
	for i in range(len(q)):
		if abs(q[i]) > numpy.pi:
			if q[i] < 0:
				q[i] = 2*numpy.pi + q[i]
			else:
				q[i] = q[i] - 2*numpy.pi
			
	return(q)
	

    """ This function checks if a set of joint angles q[] creates a valid state, or 
    one that is free of collisions. The values in q[] are assumed to be values for 
    the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.current_joint_state.name
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(7)
        req.robot_state.joint_state.effort = numpy.zeros(7)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


if __name__ == '__main__':
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()
