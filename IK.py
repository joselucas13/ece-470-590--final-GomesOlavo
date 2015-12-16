#!/usr/bin/env python

# Jose Lucas Gomes Olavo 

import math
import numpy
from copy import deepcopy

from getFK import *

# Euclidian Distance
def get_dist(e,g):
	tam_e = len(e)
	tam_g = len(g)
	summ = 0
	if (tam_e == tam_g):
		for i in range (0,tam_e):
			summ = (e[i] - g[i])**2 + summ
		d = math.sqrt(summ)
	return d

# Jacobian
def get_J(d_theta, theta, side):
	e = get_FK(theta,side)
	J = numpy.zeros(shape=(len(e),len(theta)))
	for i in range (0,len(e)):
		for j in range (0,len(theta)):
			theta_new = deepcopy(theta)
			#print theta
			theta_new[j] = theta_new[j] + d_theta
			#print theta_new-theta
			de = numpy.subtract(get_FK(theta_new,side),get_FK(theta,side))
			J[i,j] = de[i]/d_theta
	return J

# Get next Point
def get_next_point_delta(e,g,stp):
	m = g - e
	#print m
	m = m/numpy.linalg.norm(m)
	#print m
	p = m*stp
	#print p
	return p

def wrapTo2Pi(ang):
    return ang % (2*math.pi)

