import math
import numpy as np
import mujoco
BASE='Redesign_Rev_C_-_29Oct_printed_P17_1__4_Base_tray_frame_1'
CPR=9600/(2*math.pi)
S={}
def clip(v,a,b): return max(a,min(b,v))
def length(q):
 px,py=.000002,.107003;dx=-.035-px;dy=.213-py
 return math.hypot(px+dx*math.cos(q)-dy*math.sin(q)+.035,py+dx*math.sin(q)+dy*math.cos(q)-.102)
def q_for(target):
 lo,hi=-1.2,1.2
 for _ in range(45):
  m=(lo+hi)/2
  if length(m)>target:lo=m
  else:hi=m
 return (lo+hi)/2
QMIN,QMAX=q_for(.112),q_for(.082)
def reset(model,data,scenario):
 global S
 S=dict(state='CALIBRATE',t0=0.,hits=0,n=0,sum=0.,ps=0.,pn=0,flat=0.,was=False,flat_since=0.,left0=0.,right0=0.,last=-1.,ping=-1.,up=0.,fwd=0.,vel=None,integ=[0.,0.],hinge=0.,press=1,events=[(0.,'CALIBRATE')],timeouts=[],done_at=None,hall=0,speed_tick=-1.,speed_prev=None,speed_est=[0.,0.])
def enter(s,t):
 S['state']=s;S['t0']=t;S['hits']=0;S['events'].append((round(t,2),s))
def control(model,data,scenario):
 global S
 if not S or data.time < S['last']-1e-6:reset(model,data,scenario)
 t=data.time
 # Firmware runs at 100 Hz; keep last command between ticks.
 if t < S['last']+.00999:return
 dt=.01 if S['last']<0 else min(.03,t-S['last']);S['last']=t
 def pos(n):return float(data.joint(n).qpos[0])
 def vel(n):return float(data.joint(n).qvel[0])
 def act(n,v):data.ctrl[model.actuator(n).id]=v
 mat=data.body(BASE).xmat.reshape(3,3); pitch=math.atan2(mat[2,0],math.hypot(mat[0,0],mat[1,0]))
 # World-frame linear acceleration, measured at base body origin; same filter as MidBodyAccel.
 vv=np.zeros(6);mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,model.body(BASE).id,vv,0)
 v=vv[3:].copy();aa=np.zeros(3) if S['vel'] is None else (v-S['vel'])/dt;S['vel']=v
 k=dt/(1/(2*math.pi*15)+dt);f=mat[:,0].copy();f[2]=0;nf=np.linalg.norm(f);f=f/max(nf,1e-9)
 S['up']+=k*(aa[2]-S['up']);S['fwd']+=k*(float(aa@f)-S['fwd'])
 # Down-facing sensor fixed to front of base tray, reading every 60ms, excludes robot geoms.
 new=t>=S['ping']+.05999; rng=None
 if new:
  S['ping']=t; sid=model.site('range_origin').id; p=data.site_xpos[sid]; direction=-mat[:,1].copy();groups=np.array([1,0,0,0,0,0],dtype=np.uint8);gid=np.array([-1],dtype=np.int32)
  r=mujoco.mj_ray(model,data,p,direction,groups,1,-1,gid)
  if r>=0 and r<=1.:rng=r*100
 qa,qb=pos('rA0'),pos('rB0'); # forward = negative angular speed
 if t>=S['speed_tick']+.01999:
  if S['speed_prev'] is not None:
   delta=t-S['speed_tick']
   for j,q in enumerate((qa,qb)):
    sample=-(q-S['speed_prev'][j])*CPR/max(delta,1e-6)
    S['speed_est'][j]+=.5*(sample-S['speed_est'][j])
  S['speed_tick']=t;S['speed_prev']=(qa,qb)
 if pos('Track_hinge_-_front_up')>=-.005:S['hall']+=1
 else:S['hall']=0
 ca,cb=-qa*CPR,-qb*CPR
 state=S['state'];elapsed=t-S['t0']; mode=scenario.get('mode','down')
 def timeout(label): S['timeouts'].append((round(t,2),state,label))
 def creep():
  S['left0']=ca;S['right0']=cb;enter('CREEP',t)
 if mode=='down':
  if state=='CALIBRATE':
   if rng is not None:S['sum']+=rng;S['n']+=1
   S['ps']+=math.degrees(-pitch);S['pn']+=1
   if (S['n']>=10 and S['pn']>0) or elapsed>2:
    S['flat']=S['ps']/S['pn'];S['floor']=S['sum']/S['n'] if S['n'] else 0
    enter('APPROACH' if S['n']>=10 else 'BLIND',t)
  elif state=='APPROACH':
   if rng is not None:S['hits']=S['hits']+1 if rng>S['floor']+2 else 0
   if S['hits']>=2:creep()
   elif elapsed>60:timeout('no stair edge found');creep()
  elif state=='BLIND':
   if elapsed>=3:creep()
  elif state=='CREEP':
   if ((ca-S['left0'])+(cb-S['right0']))/2/849>=9:enter('HINGE_DOWN',t)
   elif elapsed>30:timeout('creep distance not reached');enter('HINGE_DOWN',t)
  elif state=='HINGE_DOWN':
   if elapsed>=.3:S['hits']=S['hits']+1 if S['up']>1.5 or S['fwd']>1.5 else 0
   if S['hits']>=2:S['hinge']=0;enter('STRAIGHTEN',t)
   elif elapsed>5:timeout('no landing felt');S['hinge']=0;enter('STRAIGHTEN',t)
  elif state=='STRAIGHTEN':
   if abs(math.degrees(-pitch)-S['flat'])>3:S['was']=True
   if S['hall']>=3:
    S['hinge']=0;S['flat_since']=t;enter('CRUISE',t)
   elif elapsed>8:timeout('hinge did not reach Hall');S['hinge']=0;S['flat_since']=t;enter('CRUISE',t)
  elif state=='CRUISE':
   if abs(math.degrees(-pitch)-S['flat'])>3:S['was']=True;S['flat_since']=t
   elif S['was'] and t-S['flat_since']>=.5:enter('DONE',t);S['done_at']=t
   if S['state']=='CRUISE' and elapsed>20:timeout('never flat again');enter('DONE',t);S['done_at']=t
  elif state=='DONE' and t-S['done_at']>=.5 and (data.body(BASE).xpos[0]>.12 or data.body(BASE).xpos[2]>.03):
   # Operator presses D again after observing the robot has not reached bottom.
   S.update(n=0,sum=0.,ps=0.,pn=0,was=False,press=S['press']+1,hinge=0.)
   enter('CALIBRATE',t)
  state=S['state']; speed={'APPROACH':1500,'BLIND':1500,'CREEP':450,'STRAIGHTEN':750,'CRUISE':1000}.get(state,0.)
  ht=-1 if state=='HINGE_DOWN' else 1 if state=='STRAIGHTEN' else 0
 else:
  edge=.345+7*.06*math.cos(math.radians(scenario['angle']))
  speed=-1000 if data.body(BASE).xpos[0]<edge+.23 else 0
  ht=1 if pos('Track_hinge_-_front_up')<-.005 else 0
 # velocity PI, counts/s, 0.05 and 0.2 PWM gains from app_config.
 for i,n in enumerate(('A','B')):
  measured=S['speed_est'][i];err=speed-measured
  S['integ'][i]=clip(S['integ'][i]+err*dt,-1275,1275)
  pwm=clip((.05*err+.2*S['integ'][i])/255,-1,1)
  if speed==0: pwm=0.; S['integ'][i]=0.
  act('drive'+n,-pwm)
 # Full PWM with firmware slew 250/s and Hall gate. Approximate DC torque-speed relation.
 if ht==0:S['hinge']=0.
 else:S['hinge']+=clip(ht-S['hinge'],-250/255*dt,250/255*dt)
 if ht>0 and S['hall']>=3:S['hinge']=0.
 w=vel('Track_hinge_-_front_up');act('hinge_motor',clip(2.45*(S['hinge']-w/.276),-2.45,2.45) if S['hinge'] else 0.)
 # Measured pin geometry and 82-112 mm physical stroke; estimated powered force and speed.
 q=clip(-pitch,QMIN,QMAX);want=clip(length(q),.082,.112)
 tid=model.tendon('l12_stroke').id;L=data.ten_length[tid];v=data.ten_velocity[tid]
 desired=clip(5*(want-L),-.01,.01);force=clip(3500*(desired-v),-50,50)
 if L<=.082 and force<0:force=0
 if L>=.112 and force>0:force=0
 act('l12',force)
