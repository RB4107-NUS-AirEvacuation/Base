import math, json, mujoco
from PIL import Image
import owner_controller as c
for a,mode in [(24,'down'),(32,'down'),(40,'down'),(24,'up'),(40,'up')]:
 m=mujoco.MjModel.from_xml_path(f'owner_{mode}_{a}.xml');d=mujoco.MjData(m);mujoco.mj_forward(m,d);c.reset(m,d,{'mode':mode,'angle':a})
 renderer=mujoco.Renderer(m,320,560);cam=mujoco.MjvCamera();cam.distance=1.25;cam.azimuth=90;cam.elevation=-12
 frames=[];worst=0;mn=1;mx=0;hist=[]
 for i in range(60000):
  c.control(m,d,{'mode':mode,'angle':a});mujoco.mj_step(m,d)
  if i%10==0:
   z=d.body('seat_level_marker').xmat[8];worst=max(worst,math.degrees(math.acos(max(-1,min(1,z)))));mn=min(mn,d.ten_length[0]);mx=max(mx,d.ten_length[0])
  if i%10000==0:
   p=d.body(c.BASE).xpos.copy();cam.lookat[:]=[p[0],0,p[2]+.1];renderer.update_scene(d,camera=cam);frames.append(Image.fromarray(renderer.render()).convert('P',palette=Image.Palette.ADAPTIVE));hist.append([round(d.time,1),*[round(float(x),3) for x in p],c.S['state']])
 frames[0].save(f'owner_{mode}_{a}.gif',save_all=True,append_images=frames[1:],duration=300,loop=0);renderer.close()
 print(json.dumps(dict(angle=a,mode=mode,end=d.body(c.BASE).xpos.tolist(),worst=worst,stroke=[mn,mx],events=c.S['events'],timeouts=c.S['timeouts'],press=c.S['press'],hist=hist,warnings=[w.number for w in d.warning])),flush=True)
