#!/usr/bin/env python

# Jose Lucas Gomes Olavo

import math
import numpy

# Get Position
def get_Pos(T):
	
	x = T[0,3]
	y = T[1,3]
	z = T[2,3]

	thetax = math.atan2(T[2,1],T[2,2])
	thetay = math.atan2(-T[2,0],(T[2,1]**2 + T[2,2]**2))
	thetaz = math.atan2(T[1,0],T[0,0])
	
	#print [x,y,z,thetax,thetay,thetaz]
	return [x,y,z,thetax,thetay,thetaz]
	
