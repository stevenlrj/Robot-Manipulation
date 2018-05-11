#!/usr/bin/env python

# Columbia Engineering
# MECS 4603 - Fall 2017

import math
import numpy
import time
import matplotlib.pyplot as plt
from state_estimator.msg import SensorData
import numpy
from geometry_msgs.msg import Pose2D
import rospy

# This class pretends to be the real robot. It generates the "true"
# information which is generally unknowable in the real world.

def range_cal(xr, yr, xl, yl):
	r = math.sqrt( (xr-xl)*(xr-xl) + (yr-yl)*(yr-yl) )
	return(r)

def bearing_cal(xr, yr, thetar, xl, yl):
	b = math.atan2(yl-yr, xl-xr) - thetar
	return(b)

def f_func(x, y, theta, vt, va, t):
	x_1 = x + t * vt * math.cos(theta)
	y_1 = y + t * vt * math.sin(theta)
	theta_1 = theta + va * t
	return(numpy.array([[x_1], [y_1], [theta_1]]))
	
def F_k(x, y, theta, vt, va, t):
	F = numpy.zeros([3, 3])
	F[0,0] = 1
	F[1,1] = 1
	F[2,2] = 1
	F[0,2] = -t*vt*math.sin(theta)
	F[1,2] = t*vt*math.cos(theta)
	return(F)

class Estimator(object):
    def __init__(self):
	self.vel_trans = 0.0
	self.vel_ang = 0.0
	self.xl = []
	self.yl = []
	self.range = []
	self.bearing = []
        self.x_est = numpy.array([[0],[0],[0]])
        self.P_est = numpy.array([[0,0,0],[0,0,0],[0,0,0]])
	self.V = numpy.array([[0.1,0,0],
                              [0,0.1,0],
			      [0,0,0.1]])
        
	self.num = 0
	self.pub_pose = rospy.Publisher("/robot_pose_estimate", Pose2D, queue_size=1)
	rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
	
    def sensor_callback(self, msg):
	x = self.x_est[0][0]
	y = self.x_est[1][0]
	theta = self.x_est[2][0]
	self.sensor_data = msg
	self.vel_trans = msg.vel_trans
	self.vel_ang = msg.vel_ang
	self.num = len(msg.readings)
	self.landmark = []
	self.range = []
	self.bearing = []
	self.xl = []
	self.yl = []
	n = self.num
	self.h = numpy.zeros([2*n, 1])
	for i in range(n):
		if self.num != 0:
			x1 = msg.readings[i].landmark.x
			y1 = msg.readings[i].landmark.y
			self.h[2*i, 0] = range_cal(x,y, x1, y1)
			self.h[2*i+1, 0] = bearing_cal(x, y, theta, x1, y1)
			self.xl.append(x1)
			self.yl.append(y1)
			self.range.append(msg.readings[i].range)
			self.bearing.append(msg.readings[i].bearing)
	
	self.W = numpy.zeros([2*self.num,2*self.num])
	for j in range(2*self.num):
		self.W[j,j] = 0.1
	self.ekf_step()

    def reset(self, x, P):
        self.x_est = x
        self.P_est = P

    def ekf_step(self):
	x = self.x_est[0][0]
	y = self.x_est[1][0]
	theta = self.x_est[2][0]
	vt = self.vel_trans
	va = self.vel_ang

	xl = self.xl
	yl = self.yl
		
	r = self.range
	b = self.bearing

	
 	t = 0.01

	if self.num == 0:
		
		F = F_k(x, y, theta, vt, va, t)
		x_pred = f_func(x, y, theta, vt, va, t)
        	P_pred = numpy.dot(F, numpy.dot(self.P_est, numpy.transpose(F))) + self.V
        	
        	self.x_est = x_pred
        	self.P_est = P_pred
		
	elif self.num > 0:
		Y = self.construct_y(r, b)
		F = F_k(x, y, theta, vt, va, t)
 		H = self.H_kplusone(x, y, theta, xl, yl)
		
		x_pred = f_func(x, y, theta, vt, va, t)
        	P_pred = numpy.dot(F, numpy.dot(self.P_est, numpy.transpose(F))) + self.V
		
        	innov = Y - self.h
        	S = numpy.dot(H, numpy.dot(P_pred, numpy.transpose(H))) + self.W
        	R = numpy.dot(numpy.dot(P_pred, numpy.transpose(H)), numpy.linalg.pinv(S))
        
        	delta_x = numpy.dot(R, innov)
        	self.x_est = x_pred + delta_x
        	delta_P = -1.0 * numpy.dot(R, numpy.dot(H,P_pred) )
        	self.P_est = P_pred + delta_P

	self.publish_pose()

    def construct_y(self, r, b):
	n = self.num
	y = numpy.zeros([2*n, 1])
	for i in range(n):
		y[2*i, 0] = r[i]
		y[2*i+1, 0] = b[i]
	return(y)


    def H_kplusone(self, x, y, theta, xl, yl):
	n = self.num
	H = numpy.zeros([2*n, 3])
	for i in range(n):
		H[2*i,0] = (x - xl[i]) / math.sqrt((x - xl[i])**2+(y - yl[i])**2)
		H[2*i,1] = (y - yl[i]) / math.sqrt((x - xl[i])**2+(y - yl[i])**2)
		H[2*i+1,0] = (yl[i] - y) / ((x - xl[i])**2 + (y - yl[i])**2)
		H[2*i+1,1] = (x - xl[i]) / ((x - xl[i])**2 + (y - yl[i])**2)
		H[2*i+1,2] = -1
	return(H)

    def publish_pose(self):
	x_est = self.x_est
	msg = Pose2D()
        msg.x = x_est[0][0]
        msg.y = x_est[1][0]
        msg.theta = x_est[2][0]
        self.pub_pose.publish(msg)

	
    def get_estimate(self):
        return self.x_est

    def get_variance(self):
        return self.P_est

if __name__ == '__main__':
    rospy.init_node('estimator', anonymous=True)
    estimator = Estimator()
    estimator.reset(numpy.array([[0],[0],[0]]), numpy.array([[0,0,0],[0,0,0],[0,0,0]]))
    rospy.spin()


# Possible options:
# 1. Perfect robot model, no noise. Simple step tracks well.
# 2. With the addition of model error (wrong mass), simple model tracks poorly
# and error is unknown and unbounded.
# 3. Addition of model noise further worsens performance.
# 4. Going to the prediction-and-update step alleviates problems.
# 5. Adding sensor noise again makes for bad performance.
# 6. Kalman filter has much better tracking in this case. Still, note that
# position uncertainty grows forever (we are never measuring it directly).
# 7. It also works well if we remove modeling error and noise (as expected).
