import json,mujoco
import owner_controller as c
from pathlib import Path
out=[]
for a in (24,32,40):
 m=mujoco.MjModel.from_xml_path(f'owner_down_{a}.xml');d=mujoco.MjData(m);mujoco.mj_forward(m,d);scenario={'mode':'down','angle':a};c.reset(m,d,scenario)
 bottom=None
 for i in range(180000):
  c.control(m,d,scenario);mujoco.mj_step(m,d)
  p=d.body(c.BASE).xpos
  if bottom is None and p[0]<=.12 and p[2]<=.03:bottom=d.time
  if c.S['state']=='DONE' and c.S['press']>=2 and d.time-c.S['done_at']>.6:break
 out.append({'angle':a,'seconds':round(d.time,2),'bottom_first_s':round(bottom,2) if bottom else None,'final_position':d.body(c.BASE).xpos.tolist(),'presses':c.S['press'],'state':c.S['state'],'events':c.S['events'],'timeouts':c.S['timeouts']})
 print(json.dumps(out[-1]),flush=True)
Path('owner_extended_results.json').write_text(json.dumps(out,indent=2))
