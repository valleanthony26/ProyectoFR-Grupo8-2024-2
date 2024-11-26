#!/usr/bin/env python2
import rbdl
import numpy as np

def MatM(q):
    for i in range(ndof):
        tau_m = np.zeros(ndof)
        rbdl.InverseDynamics(modelo, q, zeros, e[i], tau_m)
        M[i] = tau_m - g
    return M

def MatC(q,dq):
    tau_c = np.zeros(ndof)
    rbdl.InverseDynamics(modelo, q, dq, zeros, tau_c)
    return tau_c - g

def Matg(q):
    rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
    return g

if __name__ == '__main__':

  # Lectura del modelo del robot a partir de URDF (parsing)
  modelo = rbdl.loadModel('lab_ws/src/ProyectoRobotica/ROBOT_URDF/urdf/ROBOT_URDF.urdf')
  # Grados de libertad
  ndof = modelo.q_size

  # Configuracion articular
  q = np.array([2.48588849,-0.31285127,-0.16005493,-0.01715127,0.40013112, 0.13598287,0,0])
  # Velocidad articular
  dq = np.zeros(ndof)
  # Aceleracion articular
  ddq = np.zeros(ndof)
  
  # Armys
  zeros = np.zeros(ndof)          # Vector de ceros
  tau   = np.zeros(ndof)          # Para torque
  g     = np.zeros(ndof)          # Para la gravedad
  c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
  M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
  e     = np.eye(ndof)            # Vector identidad
  
  # Torque dada la configuracion del robot
  rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
  g=Matg(q)
  C=MatC(q,dq)
  M=MatM(q); 
  print('M:',M)
  print('C:',C)
  print('g:',g)

  # Parte 2: Calcular M y los efectos no lineales b usando las funciones
  # CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
  # en los arreglos llamados M2 y b2
  b2 = np.zeros(ndof)          # Para efectos no lineales
  M2 = np.zeros([ndof, ndof])  # Para matriz de inercia
  rbdl.CompositeRigidBodyAlgorithm(modelo, q, M2)

  rbdl.NonlinearEffects(modelo, q, dq, b2)
