#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */

#Code modified from hubo-simple-demo-python by Dr.Laforo

import hubo_ach as ha
import ach
import sys
import time
from ctypes import *
import math

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)

# x is 0 
y = 0.2 #0.2 for assignment
d1 = 0.4
d2 = 0.4 
# d1 + d2 = 0.8 which is height of robots waist from 

def calc_theta_2(y,d1,d2):
	numerator = y**2 - d1**2 - d2**2
	denominator = 2 * d1 * d2
	theta2 = math.acos(float(numerator)/denominator)
	return theta2

def calc_theta_1(y, d1, d2, theta2):
	numerator = y * (d1 + d2*math.cos(theta2))
	denominator = y*(d2*math.sin(theta2))
	theta1 = math.atan2(numerator,denominator)
	return theta1

theta2 = calc_theta_2(y,d1,d2)
theta1 = calc_theta_1(y,d1,d2,theta2)
theta2b = math.pi - theta2
factor = theta2b/theta1

theta2b = calc_theta_2(y2,d1,d2)
theta1b = cal_theta_1(y2,d1,d2,theta2)
theta2bb = math.pi - theta2b
factor = theta2bb/theta1

# calculates the x-distance of a 3dof system
def three_dof(theta_1,theta_2,theta_3):
	length = length_1*np.cos(theta_1)+length_2*np.cos(theta_2 + theta_3)+length_3
	return length

mode = True

# hubo spreads his arm to help balance it
def spread_arm():
	if(mode):
		i = 0.1
		while i < 1:
			ref.ref[ha.RSR] = -i
			r.put(ref)
			i += 0.01
			time.sleep(0.05)
		print("ROBOT - spread_arms() first done")
	else:
		i = 0.1
		while i < 1:
			ref.ref[ha.LSR] = i
			r.put(ref)
			i += 0.01
			time.sleep(0.05)
		print("ROBOT - spread_arms() second done")

#shift the center of mass over the right
def shift_weight():
	if(mode):
		i = 0.01
		while i < 0.15:
			ref.ref[ha.RHR] = i
			ref.ref[ha.LHR] = i
			ref.ref[ha.RAR] = -i
			ref.ref[ha.LAR] = -i
			i += 0.005
			r.put(ref)
			time.sleep(.1)
		print("ROBOT - shift_weight() first done")
	else:
		i = 0.01
		while i < 0.15:
			ref.ref[ha.RHR] = -i
			ref.ref[ha.LHR] = -i
			ref.ref[ha.RAR] = i
			ref.ref[ha.LAR] = i
			i += 0.005
			r.put(ref)
			time.sleep(.1)
		print("ROBOT - shift_weight() second done")


#balance on one foot
def lift_foot():
	if(mode):
		i = 0.01
		while i < 1.3:
			ref.ref[ha.LHP] = -i
			ref.ref[ha.LAP] = -i
			ref.ref[ha.LKN] = 2*i
			i += 0.01
			r.put(ref)
			time.sleep(.05)
		print("ROBOT - lift_foot() first done")
	else:
		i = 0.01
		while i < 1.3:
			ref.ref[ha.RHP] = -i
			ref.ref[ha.RAP] = -i
			ref.ref[ha.RKN] = 2*i
			i += 0.01
			r.put(ref)
			time.sleep(.05)
		print("ROBOT = lift_foot() second done")

def crouch_and_balance():
	if(mode):
		i = 0
		while i < theta1:
			ref.ref[ha.RAP] -= i
			ref.ref[ha.RKN] += factor*i
			ref.ref[ha.RHP] -= i
			r.put(ref)
			i += 0.05
			time.sleep(.1)
		print("ROBOT - crouch() first  done")
		time.sleep(10)
		val = ref.ref[ha.RAP]
		i = 0
		while val < 0:
			ref.ref[ha.RAP] += i
			ref.ref[ha.RKN] -= factor*i
			ref.ref[ha.RHP] += i
			i += 0.01
			val += i
			r.put(ref)
			time.sleep(2)
		print("ROBOT - balance() first done")
	else:
		i = 0
		while i < theta1b:
			ref.ref[ha.LAP] -= i
			ref.ref[ha.LKN] += factor*i
			ref.ref[ha.LHP] -= i
			r.put(ref)
			i += 0.05
			time.sleep(.1)
		print("ROBOT - crouch() second done")
		time.sleep(10)
		val = ref.ref[ha.LAP]
		i = 0
		while val < 0:
			ref.ref[ha.LAP] += i
			ref.ref[ha.LKN] -= factor*i
			ref.ref[ha.LHP] += i
			i += 0.01
			val += i
			r.put(ref)
			time.sleep(2)
		print("ROBOT - balance() second done")

def initialize_position():
	i = 1.3
	while i > -0.01:
		if(mode):
			ref.ref[ha.LHP] = -i
			ref.ref[ha.LAP] = -i
			ref.ref[ha.LKN] = 2*i
		else:
			ref.ref[ha.RHP] = -i
			ref.ref[ha.RAP] = -i
			ref.ref[ha.RKN] = 2*i
		i -= 0.01
		r.put(ref)
		time.sleep(.1)
	time.sleep(2)
	ref.ref[ha.RSR] = 0
	ref.ref[ha.LHP] = 0
	ref.ref[ha.LAP] = 0
	ref.ref[ha.LKN] = 0
	ref.ref[ha.LSR] = 0
	ref.ref[ha.RHP] = 0
	ref.ref[ha.RAP] = 0
	ref.ref[ha.RKN] = 0
	ref.ref[ha.RHR] = 0
	ref.ref[ha.LHR] = 0
	ref.ref[ha.RAR] = 0
	ref.ref[ha.LAR] = 0
	r.put(ref)
	time.sleep(1)
	print("ROBOT - initialize_position() done")
	
i = 0
while i<2:
	spread_arm()
	time.sleep(10)
	shift_weight()
	time.sleep(20)
	lift_foot()
	time.sleep(5)
	crouch_and_balance()
	time.sleep(10)
	crouch_and_balance()
	time.sleep(10)
	initialize_position()
	time.sleep(30)
	mode = False
	i+=1

print("ROBOT-sim done")
	
