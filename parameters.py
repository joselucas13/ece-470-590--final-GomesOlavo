#!/usr/bin/env python

# Jose Lucas Gomes Olavo 

import math
import numpy

# Dimensions of the Robot Hubo DRC

# Dimensions for the legs
l1 = 0.085
l2 = 0.33
l3 = 0.33
l4 = 0.12

# Dimensions for the arms
l5 = 0.38
l6 = 0.24
l7 = 0.31
l8 = 0.31
l9 = 0.13

# Structure
class stru:
    def __init__(self):
        self.x = 0
        self.y = 0
	self.z = 0
	self.thetax = 0
	self.thetay = 0
	self.thetaz = 0


##### LEG LEFT SIDE ########

# K_LHY
K_LHY = stru()
K_LHY.y = l1
K_LHY.thetaz = 1

# K_LHR
K_LHR = stru()
K_LHR.thetax = 1

# K_LHP
K_LHP = stru()
K_LHP.thetay = 1

# K_LKN
K_LKN = stru()
K_LKN.z = -l2
K_LKN.thetay = 1

# K_LAP
K_LAP = stru()
K_LAP.z = -l3
K_LAP.thetay = 1

# K_LAR
K_LAR = stru()
K_LAR.thetax = 1

# K_LEDL
K_LEDL = stru()
K_LEDL.z = -l4


##### LEG RIGHT SIDE ########


# K_RHY
K_RHY = stru()
K_RHY.y = -l1
K_RHY.thetaz = 1

# K_RHR
K_RHR = stru()
K_RHR.thetax = 1

# K_RHP
K_RHP = stru()
K_RHP.thetay = 1

# K_RKN
K_RKN = stru()
K_RKN.z = -l2
K_RKN.thetay = 1

# K_RAP
K_RAP = stru()
K_RAP.z = -l3
K_RAP.thetay = 1

# K_RAR
K_RAR = stru()
K_RAR.thetax = 1

# K_REDR
K_REDR = stru()
K_REDR.z = -l4


#####  ARM RIGHT SIDE ########

# K_RSP
K_RSP = stru()
K_RSP.z = l5
K_RSP.y = -l6
K_RSP.thetay = 1

# K_RSR
K_RSR = stru()
K_RSR.thetax = 1

# K_RSY
K_RSY = stru()
K_RSY.thetaz = 1

# K_REB
K_REB = stru()
K_REB.z = -l7
K_REB.thetay = 1

# K_RWY
K_RWY = stru()
K_RWY.thetaz = 1

# K_RWP
K_RWP = stru()
K_RWP.z = -l8
K_RWP.thetay = 1

# K_RWR
K_RWR = stru()
K_RWR.thetaz = 1

# K_REDR
K_REDR = stru()
K_REDR.z = -l9

#####  ARM LEFT SIDE ########

# K_LSP
K_LSP = stru()
K_LSP.z = l5
K_LSP.y = l6
K_LSP.thetay = 1

# K_LSR
K_LSR = stru()
K_LSR.thetax = 1

# K_LSY
K_LSY = stru()
K_LSY.thetaz = 1

# K_LEB
K_LEB = stru()
K_LEB.z = -l7
K_LEB.thetay = 1

# K_LWY
K_LWY = stru()
K_LWY.thetaz = 1

# K_LWP
K_LWP = stru()
K_LWP.z = -l8
K_LWP.thetay = 1

# K_LWR
K_LWR = stru()
K_LWR.thetaz = 1

# K_LEDR
K_LEDR = stru()
K_LEDR.z = -l9


