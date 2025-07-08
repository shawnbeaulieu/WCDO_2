import matplotlib.pyplot as plt
import WCDO.constants as c
import pyrosim.pyrosim as pyrosim
import random
import numpy as np
import pybullet as p
import pybullet_data
import imageio_ffmpeg

def addElectricField(numSeconds,electricField):

   simulateCells(numSeconds,motilityStrength=0,attractionStrength=c.attractionStrength,electricField=electricField)

def addLoneliness(numSeconds):

   simulateCells(numSeconds,motilityStrength=c.motilityStrength,attractionStrength=c.attractionStrength)

def addMotility(numSeconds):

   simulateCells(numSeconds,motilityStrength=c.motilityStrength)

def scoreElectricField(numSeconds,electricField):

   xs, ys = simulateAndTrackCells(numSeconds,motilityStrength=0,attractionStrength=c.attractionStrength,electricField=electricField)
   print(xs,ys)

def captureFrame(t,vid):

   if t%20==0:
      c.cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(c.cam_target_pos, c.cam_distance, c.cam_yaw, c.cam_pitch, c.cam_roll, c.cam_up_axis_idx)
      c.cam_projection_matrix = p.computeProjectionMatrixFOV(c.cam_fov, c.cam_width*1./c.cam_height, c.cam_near_plane, c.cam_far_plane)
      image = p.getCameraImage(c.cam_width, c.cam_height,c.cam_view_matrix, c.cam_projection_matrix)[2][:, :, :3]
      vid.send(np.ascontiguousarray(image))
      #c.cam_yaw = c.cam_yaw + 1

def createElectricField(electricField):

   x    = np.linspace(-c.petriDishWidth/2, c.petriDishWidth/2, 30)
   y    = np.linspace(-c.petriDishWidth/2, c.petriDishWidth/2, 30)
   X, Y = np.meshgrid(x, y)

   fX   =      electricField[0] * X
   fX   = fX + electricField[1] * Y
   fX   = fX + electricField[2] * np.sin(X/(c.petriDishWidth/2) * 2.0 * 3.14159)
   fX   = fX + electricField[3] * np.sin(Y/(c.petriDishWidth/2) * 2.0 * 3.14159)

   fY   =      electricField[4] * X
   fY   = fY + electricField[5] * Y
   fY   = fY + electricField[6] * np.sin(X/(c.petriDishWidth/2) * 2.0 * 3.14159)
   fY   = fY + electricField[7] * np.sin(Y/(c.petriDishWidth/2) * 2.0 * 3.14159)

   magnitude = np.sqrt(fX**2 + fY**2)
   fX = fX / (magnitude + 1e-8)
   fY = fY / (magnitude + 1e-8)

   fig, ax = plt.subplots(figsize=(1.5,1.5),dpi=600)

   q = ax.quiver(X, Y, fX, fY, magnitude, cmap='coolwarm', scale=20, alpha=1.0, width=0.003)

   plt.axis('off')

   plt.show()

def createRandomElectricField():

   electricField = np.random.choice(range(2), 8)
   x    = np.linspace(-c.petriDishWidth/2, c.petriDishWidth/2, 30)
   y    = np.linspace(-c.petriDishWidth/2, c.petriDishWidth/2, 30)
   X, Y = np.meshgrid(x, y)

   fX   =      electricField[0] * X
   fX   = fX + electricField[1] * Y
   fX   = fX + electricField[2] * np.sin(X/(c.petriDishWidth/2) * 2.0 * 3.14159)
   fX   = fX + electricField[3] * np.sin(Y/(c.petriDishWidth/2) * 2.0 * 3.14159)

   fY   =      electricField[4] * X
   fY   = fY + electricField[5] * Y
   fY   = fY + electricField[6] * np.sin(X/(c.petriDishWidth/2) * 2.0 * 3.14159)
   fY   = fY + electricField[7] * np.sin(Y/(c.petriDishWidth/2) * 2.0 * 3.14159)

   magnitude = np.sqrt(fX**2 + fY**2)
   fX = fX / (magnitude + 1e-8)
   fY = fY / (magnitude + 1e-8)

   fig, ax = plt.subplots(figsize=(1.5,1.5),dpi=600)

   q = ax.quiver(X, Y, fX, fY, magnitude, cmap='coolwarm', scale=20, alpha=1.0, width=0.003)

   plt.axis('off')
   plt.show()
   return(electricField)

def intervene(objectIDs,electricField):

   for objID in objectIDs:

      pos, orientation = p.getBasePositionAndOrientation(objID)

      x = pos[0]
      y = pos[1]

      fx   =      electricField[0] * x
      fx   = fx + electricField[1] * y
      fx   = fx + electricField[2] * np.sin(x/(c.petriDishWidth/2) * 2.0 * 3.14159)
      fx   = fx + electricField[3] * np.sin(y/(c.petriDishWidth/2) * 2.0 * 3.14159)

      fy   =      electricField[4] * x
      fy   = fy + electricField[5] * y
      fy   = fy + electricField[6] * np.sin(x/(c.petriDishWidth/2) * 2.0 * 3.14159)
      fy   = fy + electricField[7] * np.sin(y/(c.petriDishWidth/2) * 2.0 * 3.14159)

      p.applyExternalForce(objID, -1, [ fx , fy , 0 ], [0, 0, 0], p.WORLD_FRAME)

def prep():

   physicsClient = p.connect(p.DIRECT)
   p.setAdditionalSearchPath(pybullet_data.getDataPath())
   # p.setGravity(0, 0, -10)
   plane_id = p.loadURDF("plane.urdf")

   vid = imageio_ffmpeg.write_frames('vid.mp4', (c.cam_width, c.cam_height), fps=30)
   vid.send(None) # The first frame of the video must be a null frame.

   objectIDs = p.loadSDF("box.sdf")

   for objID in objectIDs:

      p.changeVisualShape(objID, -1, rgbaColor=[random.random(), random.random(), random.random(), 1])

   return vid,objectIDs

def pullTogether(objectIDs,attractionStrength):

   for objID in objectIDs:

      pos, orientation = p.getBasePositionAndOrientation(objID)

      x = 2 * attractionStrength * pos[0] - attractionStrength
      y = 2 * attractionStrength * pos[1] - attractionStrength
      z = 2 * attractionStrength * pos[2] - attractionStrength

      p.applyExternalForce(objID, -1, [ -x , -y , -z ], [0, 0, 0], p.WORLD_FRAME)

def push(objectIDs,motilityStrength):

   for objID in objectIDs:

      x = 2 * motilityStrength * random.random() - motilityStrength
      y = 2 * motilityStrength * random.random() - motilityStrength
      z = 2 * motilityStrength * random.random() - motilityStrength

      p.applyExternalForce(objID, -1, [x,y,z], [0, 0, 0], p.WORLD_FRAME)

def simulateCells(numSeconds, motilityStrength = 0 , attractionStrength = 0 , electricField = None):

   vid, objectIDs = prep()
 
   for t in range(0,625*numSeconds):

      if motilityStrength>0:

         push(objectIDs,motilityStrength)

      if attractionStrength>0:

         pullTogether(objectIDs,attractionStrength)

      if electricField != None:

         intervene(objectIDs,electricField)

      captureFrame(t,vid)

      p.stepSimulation()

   terminate_gracefully(vid)

def simulateAndTrackCells(numSeconds, motilityStrength = 0 , attractionStrength = 0 , electricField = None):

   vid, objectIDs = prep()
 
   for t in range(0,625*numSeconds):

      if motilityStrength>0:

         push(objectIDs,motilityStrength)

      if attractionStrength>0:

         pullTogether(objectIDs,attractionStrength)

      if electricField != None:

         intervene(objectIDs,electricField)

      captureFrame(t,vid)

      p.stepSimulation()

   xs = []
   ys = []
   for objID in objectIDS:
      pos, orientation = p.getBasePositionAndOrientation(objID)
      xs.append(pos[0])
      ys.append(pos[1])
      
   terminate_gracefully(vid)
   return(xs,ys)

def sprinkleCells(numCells):

   pyrosim.Start_SDF("box.sdf")

   for i in range(0,numCells):

      x = c.petriDishWidth * random.random() - (c.petriDishWidth / 2.0)
      y = c.petriDishWidth * random.random() - (c.petriDishWidth / 2.0)
      z = c.cellRadius

      pyrosim.Send_Sphere(name="Sphere", pos=[x,y,z] , radius=c.cellRadius)

   pyrosim.End()

def terminate_gracefully(vid):

   vid.close()
   p.disconnect()
