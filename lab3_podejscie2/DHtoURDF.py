#!/usr/bin/env python

import json
import math
import mathutils
import numpy

with open('DH.json','r') as file:
	params = json.loads(file.read())

with open('URDF.yaml','w') as file:
	for key in params.keys():

		a, d, alpha, theta = params[key]
		a = float(a)
		d = float(d)
		alpha = float(alpha)
		theta = float(theta)

		rot_x = mathutils.Matrix.Rotation(alpha,4,'X')
		trans_x = mathutils.Matrix.Translation((a,0,0))
		rot_z = mathutils.Matrix.Rotation(theta,4,'Z')
		trans_z = mathutils.Matrix.Translation((0,0,d))

		dh = trans_x @ rot_x @ rot_z @ trans_z
		rpy = dh.to_euler()
		xyz = dh.to_translation()

		file.write(key + ":\n")
		file.write("  j_xyz: "+str(xyz[0])+" "+str(xyz[1])+" "+str(xyz[2])+"\n")
		file.write("  j_rpy: "+str(rpy[0])+' '+str(rpy[1])+' '+str(rpy[2])+'\n')
		if ( key == "i3" ):
			file.write("  x: "+str(0.1)+"\n")
			file.write("  y: "+str(0.1)+"\n")
			file.write("  z: "+str(-1.0)+"\n")
		else:
			file.write("  x: "+str(1.0)+"\n")
			file.write("  y: "+str(0.1)+"\n")
			file.write("  z: "+str(0.1)+"\n")
		file.write("  l_rpy: "+str(0)+' '+str(0)+' '+str(0)+'\n')
