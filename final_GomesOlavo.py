#!/usr/bin/env python

# Jose Lucas Gomes Olavo 

import hubo_ach as ha
import ach
import sys
import time
from ctypes import *

from getFK import *
from IK import *

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

#########   Simulated 'Real Time' sleep ##################

fs = 10.0
Ts = 1/fs

def Sim_Sleep(ts, Ts):
	t = ts + Ts
	s.get(state, wait=False, last=False)	
	while(state.time < t):
		s.get(state, wait=True, last=False)
	return

###########################################################

############## ROBOT WALKING  ################################

# Bending the knees to walk
pos1 = 0
i = 0
while i < 5:
	s.get(state, wait=False, last=False)
	pos1 += 0.1
	ref.ref[ha.LHP] = -pos1
        ref.ref[ha.LKN] = 2*pos1
        ref.ref[ha.LAP] = -pos1
        ref.ref[ha.RHP] = -pos1
        ref.ref[ha.RKN] = 2*pos1
        ref.ref[ha.RAP] = -pos1
	r.put(ref)
	print pos1
	i += 1
	Sim_Sleep(state.time, Ts)

s.get(state, wait=False, last=False)
Sim_Sleep(state.time, 1.0)

# Shift weight to right foot
pos2 = 0
i = 0
shift = 14
while i < shift:
	s.get(state, wait=False, last=False)
	pos2 += 0.01
	ref.ref[ha.RHR] = pos2
	ref.ref[ha.LHR] = pos2
	ref.ref[ha.RAR] = -pos2
	ref.ref[ha.LAR] = -pos2
	r.put(ref)
	print pos2
	i += 1
	Sim_Sleep(state.time, Ts)

s.get(state, wait=False, last=False)
Sim_Sleep(state.time, 1.0)


# Put the left leg up
# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)

theta_LL = numpy.array([state.joint[ha.LHY].pos,state.joint[ha.LHR].pos,state.joint[ha.LHP].pos,state.joint[ha.LKN].pos,state.joint[ha.LAP].pos,state.joint[ha.LAR].pos])
#print theta_LL

current_LL = get_FK(theta_LL,'LL')
#print current_LL

# Parameters for the IK
g = numpy.array(current_LL)		# Goal Position
g[0] = g[0] + 0.05
g[2] = g[2] + 0.05
err = 0.03	  			# Maximum Error
dd_theta = 0.01 			# Delta theta
stp = 0.0001	  			# Step

count = 1.0
count2 = 0.0
while(get_dist(current_LL,g) > err):
	print get_dist(current_LL,g)
	s.get(state, wait=False, last=False)
	J = get_J(dd_theta, theta_LL,'LL')
	#print J
	Jp = numpy.linalg.pinv(J)
	#print Jp
	if ((count % 5) == 0.0):
		stp = numpy.minimum(stp*10.0,0.1)
	if (stp == 0.1):
		count2 = count2 + 1.0
		print count2
		if (count2 == 10.0):
			stp = numpy.maximum(stp/10.0,0.001)
	#print stp
	#print count
	de = get_next_point_delta(current_LL,g,stp)
	#print de
	d_theta = numpy.dot(Jp,de)
	#print d_theta
	theta_LL = theta_LL + d_theta
	#print theta_LL
	#theta = wrapTo2Pi(theta)

	ref.ref[ha.LHY] = theta_LL[0]
	ref.ref[ha.LHR] = theta_LL[1]
	ref.ref[ha.LHP] = theta_LL[2]
	ref.ref[ha.LKN] = theta_LL[3]
	ref.ref[ha.LAP] = theta_LL[4]
	ref.ref[ha.LAR] = theta_LL[5]
	r.put(ref)

    	current_LL = get_FK(theta_LL,'LL')
	print current_LL

	count = count + 1
    	if (count > 20000):
		break
	
	if (get_dist(current_LL,g) < err):
		g[2] = 0.0
	Sim_Sleep(state.time, Ts)


# Shift weight to left foot
pos3 = 0.01
i = 0
shift = 14
while i < 2*shift:
	s.get(state, wait=False, last=False)
	pos2 += 0.01
	ref.ref[ha.RHR] = ref.ref[ha.RHR] - pos3
	ref.ref[ha.LHR] = ref.ref[ha.LHR] - pos3
	ref.ref[ha.RAR] = ref.ref[ha.RAR] + pos3
	ref.ref[ha.LAR] = ref.ref[ha.LAR] + pos3
	r.put(ref)
	print pos3
	i += 1
	Sim_Sleep(state.time, Ts)


s.get(state, wait=False, last=False)
Sim_Sleep(state.time, 1.0)


# Put the right leg up
# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)

theta_RL = numpy.array([state.joint[ha.RHY].pos,state.joint[ha.RHR].pos,state.joint[ha.RHP].pos,state.joint[ha.RKN].pos,state.joint[ha.RAP].pos,state.joint[ha.RAR].pos])
#print theta_RL

current_RL = get_FK(theta_RL,'RL')
#print current_RL

# Parameters for the IK
g = numpy.array(current_RL)		# Goal Position
g[0] = g[0] + 0.05
g[2] = g[2] + 0.05
err = 0.03	  			# Maximum Error
dd_theta = 0.01 			# Delta theta
stp = 0.0001	  			# Step

count = 1.0
count2 = 0.0
while(get_dist(current_RL,g) > err):
	print get_dist(current_RL,g)
	s.get(state, wait=False, last=False)
	J = get_J(dd_theta, theta_RL,'RL')
	#print J
	Jp = numpy.linalg.pinv(J)
	#print Jp
	if ((count % 5) == 0.0):
		stp = numpy.minimum(stp*10.0,0.1)
	if (stp == 0.1):
		count2 = count2 + 1.0
		print count2
		if (count2 == 10.0):
			stp = numpy.maximum(stp/10.0,0.001)
	#print stp
	#print count
	de = get_next_point_delta(current_RL,g,stp)
	#print de
	d_theta = numpy.dot(Jp,de)
	#print d_theta
	theta_RL = theta_RL + d_theta
	#print theta_RL
	#theta = wrapTo2Pi(theta)

	ref.ref[ha.RHY] = theta_RL[0]
	ref.ref[ha.RHR] = theta_RL[1]
	ref.ref[ha.RHP] = theta_RL[2]
	ref.ref[ha.RKN] = theta_RL[3]
	ref.ref[ha.RAP] = theta_RL[4]
	ref.ref[ha.RAR] = theta_RL[5]
	r.put(ref)

    	current_RL = get_FK(theta_RL,'RL')
	print current_RL

	count = count + 1
    	if (count > 20000):
		break
	
	if (get_dist(current_RL,g) < err):
		g[2] = 0.0
	Sim_Sleep(state.time, Ts)

s.get(state, wait=False, last=False)
Sim_Sleep(state.time, 1.0)

######################################################################

######## CONTROL OF RIGHT ARM USING IK and Jacobian   ################

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)

theta_RA = numpy.array([state.joint[ha.RSP].pos,state.joint[ha.RSR].pos,state.joint[ha.RSY].pos,state.joint[ha.REB].pos,state.joint[ha.RWY].pos,state.joint[ha.RWP].pos,state.joint[ha.RWR].pos])
#print theta_RA

current_RA = get_FK(theta_RA,'RA')
#print current_RA

# Parameters for the IK
g = numpy.array([0.5,-0.20,0.25])	# Goal Position
err = 0.03	  			# Maximum Error
dd_theta = 0.01 			# Delta theta
stp = 0.00001	  			# Step

count = 1.0
count2 = 0.0
while(get_dist(current_RA,g) > err):
	print get_dist(current_RA,g)
	s.get(state, wait=False, last=False)
	J = get_J(dd_theta, theta_RA,'RA')
	#print J
	Jp = numpy.linalg.pinv(J)
	#print Jp
	if ((count % 5) == 0.0):
		stp = numpy.minimum(stp*10.0,0.1)
	if (stp == 0.1):
		count2 = count2 + 1.0
		print count2
		if (count2 == 10.0):
			stp = numpy.maximum(stp/10.0,0.001)
	#print stp
	#print count
	de = get_next_point_delta(current_RA,g,stp)
	#print de
	d_theta = numpy.dot(Jp,de)
	#print d_theta
	theta_RA = theta_RA + d_theta
	#print theta_RA
	#theta = wrapTo2Pi(theta)

	ref.ref[ha.RSP] = theta_RA[0]
	ref.ref[ha.RSR] = theta_RA[1]
	ref.ref[ha.RSY] = theta_RA[2]
	ref.ref[ha.REB] = theta_RA[3]
	ref.ref[ha.RWY] = theta_RA[4]
	ref.ref[ha.RWP] = theta_RA[5]
	ref.ref[ha.RWR] = theta_RA[6]
	r.put(ref)

    	current_RA = get_FK(theta_RA,'RA')
	print current_RA

	count = count + 1
    	if (count > 20000):
		break
	Sim_Sleep(state.time, Ts)

s.get(state, wait=False, last=False)
Sim_Sleep(state.time, 2.0)

#######################################################################


# Close the connection to the channels
r.close()
s.close()

