import pybullet as p
import time, math
import json
import argparse
import os
from pathlib import Path
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--json_file', help='Benchmark file', type=str, default='fixtures/3D/unit-tests/edge-edge.json')
parser.add_argument('--create_video', help='Create video', type=int, default=0)
parser.add_argument('--shift_mag', help='Shift the collision/visual (due to OBJ file not centered) values -1=negative shift, 0 = no shift, 1 = positive shift', type=int, default=0)
parser.add_argument('--camera_distance', help='Camera Distance (to target)', type=float, default=7)
parser.add_argument('--camera_target_x', help='Camera target (lookat) position x coordinate', type=float, default=-1)
parser.add_argument('--camera_target_y', help='Camera target (lookat) position y coordinate', type=float, default=1)
parser.add_argument('--camera_target_z', help='Camera target (lookat) position z coordinate', type=float, default=1)
parser.add_argument('--enable_sat', help='Enable Separating Axis Test (SAT) collision', type=int, default=0)
parser.add_argument('--enable_cone_friction', help='Enable Cone friction (instead of the default pyramid friction model)', type=int, default=0)
parser.add_argument('--default_friction', help='Default friction coefficient (it nof provided in json file)', type=float, default=1)




args = parser.parse_args()
print("json_path=", args.json_file)
head, tail = os.path.split(args.json_file)
name_from_jason = Path(tail)
print("json_tail = ", tail)
json_stripped_name = str(name_from_jason.with_suffix(''))
print("json_head = ", json_stripped_name)
with open(args.json_file) as f:
    data = json.load(f)


print("data=",data)
print("timestep from json=",data['timestep'])
dt = data['timestep']
rigid_body_problem = data['rigid_body_problem']
gravity = rigid_body_problem['gravity']
print("scene_type=",data['scene_type'])
print("gravity=",gravity)
#dt=1./100.
max_time_seconds = 5
if 'max_time' in data:
  max_time_seconds = data['max_time']
sim_fps = int(1./dt)
print("sim_fps=",sim_fps)
video_fps = 100

#data['scene_type'] is not unique, use json_stripped_name instead
if args.create_video:
  options="--mp4=\""+json_stripped_name+"_"+str(sim_fps)+"fps_pybullet.mp4\" --mp4fps="+str(video_fps)
  p.connect(p.GUI, options=options)
  #this line is to make sure video is frame-precise
  p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
else:
  p.connect(p.GUI)



def degree2rad(deg):
  return deg*(math.pi/180.)
  
def objectFromObj(objFileName, mass=1, meshScale=[1,1,1]):
  col_flags = p.URDF_INITIALIZE_SAT_FEATURES
  if mass==0:
     col_flags=p.GEOM_FORCE_CONCAVE_TRIMESH #| p.URDF_INITIALIZE_SAT_FEATURES 
  print("objFileName=",objFileName)
  collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName=objFileName,
                                          collisionFramePosition=[0,0,0],
                                          meshScale=meshScale,
                                          flags=col_flags
                                          )

  visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                          fileName=objFileName,
                                          visualFramePosition=[0,0,0],
                                          meshScale=meshScale
                                          ) 
  body = p.createMultiBody(baseMass=mass,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[0,0,0]
                      )
  averageVertexX=0
  averageVertexY=0
  averageVertexZ=0
  if (mass>0):
    meshData = p.getMeshData(body)
    #print("meshData=",meshData)  
    
    #compute inertial frame, assuming mass at vertices
   
    numVertices=0
    for v in meshData[1]:
      #print("v=",v)
      averageVertexX += v[0]
      averageVertexY += v[1]
      averageVertexZ += v[2]
      numVertices=numVertices+1
    if (numVertices>0):
      averageVertexX/=float(numVertices)
      averageVertexY/=float(numVertices)
      averageVertexZ/=float(numVertices)
    
    p.removeBody(body)#p.resetBasePositionAndOrientation(body, [100000,0,0],[0,0,0,1])
    shift=[-averageVertexX,-averageVertexY,-averageVertexZ]
    
    collisionShapeId2 = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName=objFileName,
                                          collisionFramePosition=shift,
                                          meshScale=meshScale
                                          ,flags=col_flags
                                          )
    visualShapeId2 = p.createVisualShape(shapeType=p.GEOM_MESH,
                                          fileName=objFileName,
                                          visualFramePosition=shift,
                                          meshScale=meshScale
                                          )                   
    
    body = p.createMultiBody(baseMass=mass,
                      baseInertialFramePosition=[0,0,0],
                      baseCollisionShapeIndex=collisionShapeId2,
                      baseVisualShapeIndex=visualShapeId2,
                      basePosition=[0,0,0]
                      )
    #make dynamics objects orange
    p.changeVisualShape(body, -1, rgbaColor=[1,120/255.,0.2,1])#255,69,0,255])                      
  p.changeVisualShape(body, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
  return body, [averageVertexX,averageVertexY,averageVertexZ]

                                     


p.configureDebugVisualizer(flag = p.COV_ENABLE_Y_AXIS_UP)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.setPhysicsEngineParameter(enableSAT=args.enable_sat, enableConeFriction=args.enable_cone_friction)

p.setGravity(gravity[0],gravity[1],gravity[2])
p.setTimeStep(dt)
#plane = p.loadURDF("plane_implicit.urdf")#, pos = objectFromObj("meshes/plane.obj", mass=0)
#p.changeDynamics(plane,-1,lateralFriction=1, frictionAnchor = 1, contactStiffness=30000, contactDamping=10000)
#orn = p.getQuaternionFromEuler([ -math.pi/2,0, 0])
#p.resetBasePositionAndOrientation(plane, [0,0,0], orn)

orn = [0,0,0,1]

shift_mag = args.shift_mag

for body in rigid_body_problem['rigid_bodies']:
  
  name = "meshes/"+body['mesh']
  print("body.mesh=", name)
  mass = 1
  
  if 'type' in body:
    if body['type']=="static":
      mass=0
  if 'is_dof_fixed' in body:
      if body['is_dof_fixed']:
        #todo: add dofs dependent on this flag
        mass=0
  
  meshScale = [1,1,1]
  #why both 'scale' and 'dimensions'? and do we need to multiply 'dimentions' by half?
  if 'scale' in body:
    meshScale = body['scale']
  if 'dimensions' in body:
    meshScale = [
      meshScale[0]*body['dimensions'][0]*0.5,
      meshScale[1]*body['dimensions'][1]*0.5,
      meshScale[2]*body['dimensions'][2]*0.5]
  body_id, com_shift = objectFromObj(name, mass=mass,meshScale=meshScale)
  
  gravity = rigid_body_problem['gravity']
	
  coefficient_friction = args.default_friction
  
  # combined_friction = friction_a * friction_b so
  # keep friction of static objects to 'default'=1
  if mass>0:
	  if 'coefficient_friction' in rigid_body_problem:
	  	coefficient_friction = rigid_body_problem['coefficient_friction']
	  if 'coefficient_friction' in body:
	    coefficient_friction = body['coefficient_friction']
    
  p.changeDynamics(body_id,-1,lateralFriction=coefficient_friction, frictionAnchor = 1)
  pos = [0,0,0]
  print("body=",body)
  if 'position' in body:
    pos = body['position']
  eul = [0,0,0]
  if 'rotation' in body:
    eul = body['rotation']
    eul = [degree2rad(eul[0]),degree2rad(eul[1]),degree2rad(eul[2])]
  lin_vel = [0,0,0]
  ang_vel = [0,0,0]
  
  if 'linear_velocity' in body:
    lin_vel = body['linear_velocity']
  if 'angular_velocity' in body:
    ang_vel_degree = body['angular_velocity']
    ang_vel = [degree2rad(ang_vel_degree[0]),degree2rad(ang_vel_degree[1]),degree2rad(ang_vel_degree[2])]
  
  orn = p.getQuaternionFromEuler(eul)
  print("com_shift=",com_shift)
  print("pos=",pos)
  #we don't need to compensate for obj files with center away from origin (as in arch?)
  
  com_pos = [pos[0]+shift_mag*com_shift[0],
             pos[1]+shift_mag*com_shift[1],
             pos[2]+shift_mag*com_shift[2]]
  #com_pos = pos
  #com_pos = [pos[0]+com_shift[0],pos[1]+com_shift[1],pos[2]+com_shift[2]]
  print("com_pos=",com_pos)
  p.resetBasePositionAndOrientation(body_id, com_pos, orn)
  p.resetBaseVelocity(body_id, lin_vel, ang_vel)


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
cameraPitch = 0.
cameraYaw = -180.
sim_frame=0.
p.resetDebugVisualizerCamera(cameraDistance=args.camera_distance,cameraYaw=cameraYaw,cameraPitch=cameraPitch,cameraTargetPosition=[args.camera_target_x,args.camera_target_y,args.camera_target_z])

video_frame=0.
vid_per_sim_frame = video_fps/sim_fps

prev_step = 0
step_id = p.addUserDebugParameter("Step", 1,-1,prev_step)
prev_run = 0
run_id = p.addUserDebugParameter("Run", 1,-1,prev_run)

run_sim = args.create_video

print("sim_fps=",sim_fps)

while p.isConnected():
  step = p.readUserDebugParameter(step_id)
  if step!=prev_step:
    prev_step = step
    p.stepSimulation()

  run = p.readUserDebugParameter(run_id)
  if run != prev_run:
    prev_run = run
    run_sim = 1-run_sim
     
    
  if run_sim:
    p.stepSimulation()
    
  sim_frame=sim_frame+1.
  
  if args.create_video and sim_frame>=max_time_seconds*sim_fps:
    break
  video_frame += vid_per_sim_frame
  #print("video_frame=",video_frame)
  if (video_frame>=1):
    video_frame-=1
    if args.create_video:
      p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #add some sleep, if we don't create a video, so we can interact at real-time speed
  #rather than super fast sim
  if not args.create_video:
    time.sleep(dt)
