#!/usr/bin/env python2
import numpy as np
from copy import copy
from pyquaternion import Quaternion
#import rbdl


cos=np.cos; sin=np.sin; pi=np.pi



class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/ROBOT_URDF.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq

def dh(d, theta, a, alpha):
    cth = cos(theta); sth = sin(theta)
    ca = cos(alpha); sa = sin(alpha)
    Tdh = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                    [sth,  ca*cth, -sa*cth, a*sth],
                    [0,        sa,     ca,      d],
                    [0,         0,      0,      1]])
    return Tdh
    
    
def fkine(q):
 T1 = dh(0.64786,q[0]-pi/2,0,pi/2)
 T2 = dh(0,pi/2-q[1],0.68572,0)
 T3 = dh(0,-pi/2-q[2],0.6,0)
 T4 = dh(0,pi/2-q[3],0,pi/2)
 T5 = dh(0.91615+q[4],pi,0,-3*pi/2)
 T6 = dh(0,-q[5],0.504,0)
 T = np.dot(np.dot(np.dot(np.dot(np.dot(T1, T2), T3), T4), T5), T6)
 return T


def jacobian(q, delta=0.0001):
 n = q.size
 J = np.zeros((3,n))
 T = fkine(q)    
 for i in range(n):
  dq = copy(q)
  dq[i] += delta
  T_inc = fkine(dq)
  J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
 return J


def jacobian_pose(q, delta=0.0001):
 n = q.size
 J = np.zeros((7,n))
 T = fkine(q)
 x = TF2xyzquat(T)
 for i in range(n):
    dq = copy(q)
    dq[i] += delta
    T_inc = fkine(dq)
    x_inc = TF2xyzquat(T_inc)
    J[:, i] = (x_inc-x) / delta    
 return J



def ikine(xdes, q0):
 epsilon  = 0.001
 max_iter = 1000

 joint1_min=-6.283
 joint1_max=6.283

 joint2_min=-1.57
 joint2_max=1.57

 joint3_min=-3.142
 joint3_max=0.785

 joint4_min=-1.57
 joint4_max=1.57

 joint6_min=-4.04
 joint6_max=0.785

 q5_min=0
 q5_max=0.4
 q  = copy(q0)
 for i in range(max_iter):
  q[0] = max(min(q[0], joint1_max), joint1_min);
  q[1] = max(min(q[1], joint2_max), joint2_min);
  q[2] = max(min(q[2], joint3_max), joint3_min);
  q[3] = max(min(q[3], joint4_max), joint4_min);
  q[4] = max(min(q[4], q5_max), q5_min);
  q[5] = max(min(q[5], joint6_max), joint6_min); 
  T=fkine(q)
  x_actual=T[0:3,3]
  error=xdes-x_actual
  if np.linalg.norm(error)<epsilon:
    break
  J=jacobian(q)
  q=q+np.dot(np.linalg.pinv(J), error)
     
 return q


def ik_gradient(xdes, q0):
 epsilon  = 0.001
 max_iter = 1000
 delta    = 0.00001

 q  = copy(q0)
 for i in range(max_iter):
  T=fkine(q)
  x_actual=T[0:3,3]
  error=xdes-x_actual
  if np.linalg.norm(error) < epsilon:
    break
  J=jacobian(q)
  q=q+delta*np.dot(J.T,error)  
    
 return q

    
def rot2quat(R):
 dEpsilon = 1e-6
 quat = 4*[0.,]

 quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
 if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
  quat[1] = 0.0
 else:
  quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
 if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
  quat[2] = 0.0
 else:
  quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
 if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
  quat[3] = 0.0
 else:
  quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

 return np.array(quat)


#def TF2xyzquat(T):
 #quat = rot2quat(T[0:3,0:3])
 #res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
 #return np.array(res)

def TF2xyzquat(T):
 quat = Quaternion(matrix=T[0:3,0:3])
 return np.array([T[0,3], T[1,3], T[2,3], quat.w, quat.x, quat.y, quat.z])







