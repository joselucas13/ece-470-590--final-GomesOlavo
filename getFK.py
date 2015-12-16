#!/usr/bin/env python

# Jose Lucas Gomes Olavo 

import math
import numpy

from parameters import *
from get_position import *

# Function to get FK
def get_FK(theta,side):
	T = get_T(theta,side)
	#print T
	[x,y,z,thetax,thetay,thetaz] = get_Pos(T)
	#print [x,y,z,thetax,thetay,thetaz]
	return [x,y,z]


# Rotation Matrix x
def R_x(theta):
	rot_x = numpy.matrix([[1,0,0],[0,math.cos(theta),-math.sin(theta)],[0,math.sin(theta),math.cos(theta)]])
	return rot_x

# Rotation Matrix y
def R_y(theta):
	rot_y = numpy.matrix([[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]])
	return rot_y

# Rotation Matrix z
def R_z(theta):
	rot_z = numpy.matrix([[math.cos(theta),-math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]])
	return rot_z

# Transformation Matrix
def get_T(theta,side):

############# LEFT SIDE LEG ##################
	if (side == 'LL'):
		R1 = K_LHY.thetax*R_x(theta[0]) + K_LHY.thetay*R_y(theta[0]) + K_LHY.thetaz*R_z(theta[0])
		T1 = numpy.matrix([[R1[0,0],R1[0,1],R1[0,2],K_LHY.x],[R1[1,0],R1[1,1],R1[1,2],K_LHY.y],[R1[2,0],R1[2,1],R1[2,2],K_LHY.z],[0,0,0,1]])	
		#print T1

		R2 = K_LHR.thetax*R_x(theta[1]) + K_LHR.thetay*R_y(theta[1]) + K_LHR.thetaz*R_z(theta[1])
		T2 = numpy.matrix([[R2[0,0],R2[0,1],R2[0,2],K_LHR.x],[R2[1,0],R2[1,1],R2[1,2],K_LHR.y],[R2[2,0],R2[2,1],R2[2,2],K_LHR.z],[0,0,0,1]])	
		#print T2

		R3 = K_LHP.thetax*R_x(theta[2]) + K_LHP.thetay*R_y(theta[2]) + K_LHP.thetaz*R_z(theta[2])
		T3 = numpy.matrix([[R3[0,0],R3[0,1],R3[0,2],K_LHP.x],[R3[1,0],R3[1,1],R3[1,2],K_LHP.y],[R3[2,0],R3[2,1],R3[2,2],K_LHP.z],[0,0,0,1]])	
		#print T3

		R4 = K_LKN.thetax*R_x(theta[3]) + K_LKN.thetay*R_y(theta[3]) + K_LKN.thetaz*R_z(theta[3])
		T4 = numpy.matrix([[R4[0,0],R4[0,1],R4[0,2],K_LKN.x],[R4[1,0],R4[1,1],R4[1,2],K_LKN.y],[R4[2,0],R4[2,1],R4[2,2],K_LKN.z],[0,0,0,1]])	
		#print T4

		R5 = K_LAP.thetax*R_x(theta[4]) + K_LAP.thetay*R_y(theta[4]) + K_LAP.thetaz*R_z(theta[4])
		T5 = numpy.matrix([[R5[0,0],R5[0,1],R5[0,2],K_LAP.x],[R5[1,0],R5[1,1],R5[1,2],K_LAP.y],[R5[2,0],R5[2,1],R5[2,2],K_LAP.z],[0,0,0,1]])	
		#print T5

		R6 = K_LAR.thetax*R_x(theta[5]) + K_LAR.thetay*R_y(theta[5]) + K_LAR.thetaz*R_z(theta[5])
		T6 = numpy.matrix([[R6[0,0],R6[0,1],R6[0,2],K_LAR.x],[R6[1,0],R6[1,1],R6[1,2],K_LAR.y],[R6[2,0],R6[2,1],R6[2,2],K_LAR.z],[0,0,0,1]])	
		#print T6

		T7 = numpy.matrix([[1,0,0,K_LEDL.x],[0,1,0,K_LEDL.y],[0,0,1,K_LEDL.z],[0,0,0,1]])
		#print T7

		T = T1*T2*T3*T4*T5*T6*T7
		#print T
		return T

######### RIGHT SIDE LEG ###############
	if (side == 'RL'):
		R1 = K_RHY.thetax*R_x(theta[0]) + K_RHY.thetay*R_y(theta[0]) + K_RHY.thetaz*R_z(theta[0])
		T1 = numpy.matrix([[R1[0,0],R1[0,1],R1[0,2],K_RHY.x],[R1[1,0],R1[1,1],R1[1,2],K_RHY.y],[R1[2,0],R1[2,1],R1[2,2],K_RHY.z],[0,0,0,1]])	
		#print T1

		R2 = K_RHR.thetax*R_x(theta[1]) + K_RHR.thetay*R_y(theta[1]) + K_RHR.thetaz*R_z(theta[1])
		T2 = numpy.matrix([[R2[0,0],R2[0,1],R2[0,2],K_RHR.x],[R2[1,0],R2[1,1],R2[1,2],K_RHR.y],[R2[2,0],R2[2,1],R2[2,2],K_RHR.z],[0,0,0,1]])	
		#print T2

		R3 = K_RHP.thetax*R_x(theta[2]) + K_RHP.thetay*R_y(theta[2]) + K_RHP.thetaz*R_z(theta[2])
		T3 = numpy.matrix([[R3[0,0],R3[0,1],R3[0,2],K_RHP.x],[R3[1,0],R3[1,1],R3[1,2],K_RHP.y],[R3[2,0],R3[2,1],R3[2,2],K_RHP.z],[0,0,0,1]])	
		#print T3

		R4 = K_RKN.thetax*R_x(theta[3]) + K_RKN.thetay*R_y(theta[3]) + K_RKN.thetaz*R_z(theta[3])
		T4 = numpy.matrix([[R4[0,0],R4[0,1],R4[0,2],K_RKN.x],[R4[1,0],R4[1,1],R4[1,2],K_RKN.y],[R4[2,0],R4[2,1],R4[2,2],K_RKN.z],[0,0,0,1]])	
		#print T4

		R5 = K_RAP.thetax*R_x(theta[4]) + K_RAP.thetay*R_y(theta[4]) + K_RAP.thetaz*R_z(theta[4])
		T5 = numpy.matrix([[R5[0,0],R5[0,1],R5[0,2],K_RAP.x],[R5[1,0],R5[1,1],R5[1,2],K_RAP.y],[R5[2,0],R5[2,1],R5[2,2],K_RAP.z],[0,0,0,1]])	
		#print T5

		R6 = K_RAR.thetax*R_x(theta[5]) + K_RAR.thetay*R_y(theta[5]) + K_RAR.thetaz*R_z(theta[5])
		T6 = numpy.matrix([[R6[0,0],R6[0,1],R6[0,2],K_RAR.x],[R6[1,0],R6[1,1],R6[1,2],K_RAR.y],[R6[2,0],R6[2,1],R6[2,2],K_RAR.z],[0,0,0,1]])	
		#print T6

		T7 = numpy.matrix([[1,0,0,K_REDR.x],[0,1,0,K_REDR.y],[0,0,1,K_REDR.z],[0,0,0,1]])
		#print T7

		T = T1*T2*T3*T4*T5*T6*T7
		#print T
		return T

############# RIGHT SIDE ARM ############
	if (side == 'RA'):
		R1 = K_RSP.thetax*R_x(theta[0]) + K_RSP.thetay*R_y(theta[0]) + K_RSP.thetaz*R_z(theta[0])
		T1 = numpy.matrix([[R1[0,0],R1[0,1],R1[0,2],K_RSP.x],[R1[1,0],R1[1,1],R1[1,2],K_RSP.y],[R1[2,0],R1[2,1],R1[2,2],K_RSP.z],[0,0,0,1]])	
		#print T1

		R2 = K_RSR.thetax*R_x(theta[1]) + K_RSR.thetay*R_y(theta[1]) + K_RSR.thetaz*R_z(theta[1])
		T2 = numpy.matrix([[R2[0,0],R2[0,1],R2[0,2],K_RSR.x],[R2[1,0],R2[1,1],R2[1,2],K_RSR.y],[R2[2,0],R2[2,1],R2[2,2],K_RSR.z],[0,0,0,1]])	
		#print T2

		R3 = K_RSY.thetax*R_x(theta[2]) + K_RSY.thetay*R_y(theta[2]) + K_RSY.thetaz*R_z(theta[2])
		T3 = numpy.matrix([[R3[0,0],R3[0,1],R3[0,2],K_RSY.x],[R3[1,0],R3[1,1],R3[1,2],K_RSY.y],[R3[2,0],R3[2,1],R3[2,2],K_RSY.z],[0,0,0,1]])	
		#print T3

		R4 = K_REB.thetax*R_x(theta[3]) + K_REB.thetay*R_y(theta[3]) + K_REB.thetaz*R_z(theta[3])
		T4 = numpy.matrix([[R4[0,0],R4[0,1],R4[0,2],K_REB.x],[R4[1,0],R4[1,1],R4[1,2],K_REB.y],[R4[2,0],R4[2,1],R4[2,2],K_REB.z],[0,0,0,1]])	
		#print T4

		R5 = K_RWY.thetax*R_x(theta[4]) + K_RWY.thetay*R_y(theta[4]) + K_RWY.thetaz*R_z(theta[4])
		T5 = numpy.matrix([[R5[0,0],R5[0,1],R5[0,2],K_RWY.x],[R5[1,0],R5[1,1],R5[1,2],K_RWY.y],[R5[2,0],R5[2,1],R5[2,2],K_RWY.z],[0,0,0,1]])	
		#print T5

		R6 = K_RWP.thetax*R_x(theta[5]) + K_RWP.thetay*R_y(theta[5]) + K_RWP.thetaz*R_z(theta[5])
		T6 = numpy.matrix([[R6[0,0],R6[0,1],R6[0,2],K_RWP.x],[R6[1,0],R6[1,1],R6[1,2],K_RWP.y],[R6[2,0],R6[2,1],R6[2,2],K_RWP.z],[0,0,0,1]])	
		#print T6

		R7 = K_RWR.thetax*R_x(theta[6]) + K_RWR.thetay*R_y(theta[6]) + K_RWR.thetaz*R_z(theta[6])
		T7 = numpy.matrix([[R7[0,0],R7[0,1],R7[0,2],K_RWR.x],[R7[1,0],R7[1,1],R7[1,2],K_RWR.y],[R7[2,0],R7[2,1],R7[2,2],K_RWR.z],[0,0,0,1]])	
		#print T7

		T8 = numpy.matrix([[1,0,0,K_REDR.x],[0,1,0,K_REDR.y],[0,0,1,K_REDR.z],[0,0,0,1]])
		#print T7

		T = T1*T2*T3*T4*T5*T6*T7*T8
		#print T
		return T

############# LEFT SIDE ARM ############
	if (side == 'LA'):
		R1 = K_LSP.thetax*R_x(theta[0]) + K_LSP.thetay*R_y(theta[0]) + K_LSP.thetaz*R_z(theta[0])
		T1 = numpy.matrix([[R1[0,0],R1[0,1],R1[0,2],K_LSP.x],[R1[1,0],R1[1,1],R1[1,2],K_LSP.y],[R1[2,0],R1[2,1],R1[2,2],K_LSP.z],[0,0,0,1]])	
		#print T1

		R2 = K_LSR.thetax*R_x(theta[1]) + K_LSR.thetay*R_y(theta[1]) + K_LSR.thetaz*R_z(theta[1])
		T2 = numpy.matrix([[R2[0,0],R2[0,1],R2[0,2],K_LSR.x],[R2[1,0],R2[1,1],R2[1,2],K_LSR.y],[R2[2,0],R2[2,1],R2[2,2],K_LSR.z],[0,0,0,1]])	
		#print T2

		R3 = K_LSY.thetax*R_x(theta[2]) + K_LSY.thetay*R_y(theta[2]) + K_LSY.thetaz*R_z(theta[2])
		T3 = numpy.matrix([[R3[0,0],R3[0,1],R3[0,2],K_LSY.x],[R3[1,0],R3[1,1],R3[1,2],K_LSY.y],[R3[2,0],R3[2,1],R3[2,2],K_LSY.z],[0,0,0,1]])	
		#print T3

		R4 = K_LEB.thetax*R_x(theta[3]) + K_LEB.thetay*R_y(theta[3]) + K_LEB.thetaz*R_z(theta[3])
		T4 = numpy.matrix([[R4[0,0],R4[0,1],R4[0,2],K_LEB.x],[R4[1,0],R4[1,1],R4[1,2],K_LEB.y],[R4[2,0],R4[2,1],R4[2,2],K_LEB.z],[0,0,0,1]])	
		#print T4

		R5 = K_LWY.thetax*R_x(theta[4]) + K_LWY.thetay*R_y(theta[4]) + K_LWY.thetaz*R_z(theta[4])
		T5 = numpy.matrix([[R5[0,0],R5[0,1],R5[0,2],K_LWY.x],[R5[1,0],R5[1,1],R5[1,2],K_LWY.y],[R5[2,0],R5[2,1],R5[2,2],K_LWY.z],[0,0,0,1]])	
		#print T5

		R6 = K_RWP.thetax*R_x(theta[5]) + K_RWP.thetay*R_y(theta[5]) + K_RWP.thetaz*R_z(theta[5])
		T6 = numpy.matrix([[R6[0,0],R6[0,1],R6[0,2],K_RWP.x],[R6[1,0],R6[1,1],R6[1,2],K_RWP.y],[R6[2,0],R6[2,1],R6[2,2],K_RWP.z],[0,0,0,1]])	
		#print T6

		R7 = K_LWR.thetax*R_x(theta[6]) + K_LWR.thetay*R_y(theta[6]) + K_LWR.thetaz*R_z(theta[6])
		T7 = numpy.matrix([[R7[0,0],R7[0,1],R7[0,2],K_LWR.x],[R7[1,0],R7[1,1],R7[1,2],K_LWR.y],[R7[2,0],R7[2,1],R7[2,2],K_LWR.z],[0,0,0,1]])	
		#print T7

		T8 = numpy.matrix([[1,0,0,K_LEDR.x],[0,1,0,K_LEDR.y],[0,0,1,K_LEDR.z],[0,0,0,1]])
		#print T7

		T = T1*T2*T3*T4*T5*T6*T7*T8
		#print T
		return T
