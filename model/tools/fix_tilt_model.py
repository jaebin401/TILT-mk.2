"""TILT URDF/MJCF post-processing: rename links, unify joint sign convention,
add joint limits, add sole frames. Source of conventions: firmware tilt_config.h
  +x forward, +y left, +z up (robot base frame). Right-hand rule, common to both sides:
  hip_roll  + : foot moves to robot's left (+y)   -> axis +x (world)
  hip_pitch + : leg swings backward               -> axis +y
  knee_pitch+ : knee flexes                       -> axis +y
  (arm/head follow the same rule: pitch about +y, roll about +x)
"""
import xml.etree.ElementTree as ET, numpy as np, math, struct, shutil, os, sys
D='tilt_description'; M='tilt_mjcf'
DEG=math.pi/180
RENAME={'assembly_1':'torso','assembly_1_2':'head',
        'hip_pitch_mount':'r_hip','thigh':'r_thigh','calf_l':'r_calf',
        'hip_pitch_mount_2':'l_hip','thigh_2':'l_thigh','calf_r':'l_calf',
        'shoulder':'r_shoulder','arm_r_assembly':'r_arm',
        'shoulder_2':'l_shoulder','arm_l_assembly':'l_arm'}
WANT_AXIS={'l_hip_roll':[1,0,0],'r_hip_roll':[1,0,0],
           'l_hip_pitch':[0,1,0],'r_hip_pitch':[0,1,0],
           'l_knee_pitch':[0,1,0],'r_knee_pitch':[0,1,0],
           'l_arm_pitch':[0,1,0],'r_arm_pitch':[0,1,0],
           'l_arm_roll':[1,0,0],'r_arm_roll':[1,0,0],'head_pitch':[0,1,0]}
# limits (deg). legs: tilt_config.h JOINT_LIMIT (roll provisional, pitch/knee mk.1). arms/head: PLACEHOLDER
LIM={'l_hip_roll':(-13,17),'r_hip_roll':(-13,17),
     'l_hip_pitch':(-92,20),'r_hip_pitch':(-92,20),
     'l_knee_pitch':(-85,76),'r_knee_pitch':(-85,76),
     'l_arm_pitch':(-90,90),'r_arm_pitch':(-90,90),
     'l_arm_roll':(-90,90),'r_arm_roll':(-90,90),'head_pitch':(-45,45)}
# effort [N m], velocity [rad/s] -- approximate datasheet values, VERIFY
ACT={'sts3215':(1.9,4.7),'sg90':(0.18,10.0),'mg996':(1.0,6.0)}
JACT={j:'sts3215' for j in ['l_hip_roll','r_hip_roll','l_hip_pitch','r_hip_pitch','l_knee_pitch','r_knee_pitch']}
JACT.update({j:'sg90' for j in ['l_arm_pitch','r_arm_pitch','l_arm_roll','r_arm_roll']}); JACT['head_pitch']='mg996'

def rpy2R(r,p,y):
    cr,sr,cp,sp,cy,sy=map(lambda f:f,(math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)))
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def R2rpy(R):
    return (math.atan2(R[2,1],R[2,2]), math.asin(max(-1,min(1,-R[2,0]))), math.atan2(R[1,0],R[0,0]))
def R2quat(R):
    w=math.sqrt(max(0,1+R[0,0]+R[1,1]+R[2,2]))/2
    x=math.copysign(math.sqrt(max(0,1+R[0,0]-R[1,1]-R[2,2]))/2,R[2,1]-R[1,2])
    y=math.copysign(math.sqrt(max(0,1-R[0,0]+R[1,1]-R[2,2]))/2,R[0,2]-R[2,0])
    z=math.copysign(math.sqrt(max(0,1-R[0,0]-R[1,1]+R[2,2]))/2,R[1,0]-R[0,1])
    return (w,x,y,z)
def H(R,p): T=np.eye(4); T[:3,:3]=R; T[:3,3]=p; return T
fl=lambda s:[float(v) for v in s.split()]

def parse(fn):
    return ET.parse(fn, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))

# ---------- URDF ----------
raw=f'{D}/robot_raw.urdf'
if not os.path.exists(raw): shutil.copy(f'{D}/robot.urdf',raw)
tree=parse(raw); r=tree.getroot()
links={l.get('name'):l for l in r.findall('link')}; joints=r.findall('joint')
child={j.find('child').get('link'):j for j in joints}
W={'assembly_1':np.eye(4)}
def world(n):
    if n not in W:
        j=child[n]; o=j.find('origin'); W[n]=world(j.find('parent').get('link'))@H(rpy2R(*fl(o.get('rpy'))),fl(o.get('xyz')))
    return W[n]
# sole frames from mesh (in original link frames, URDF zero pose)
def stl(fn):
    d=open(fn,'rb').read(); n=struct.unpack('<I',d[80:84])[0]
    return [np.array(struct.unpack('<12f',d[84+50*k:84+50*k+48])[3:]).reshape(3,3) for k in range(n)]
SOLE={}
for old,new in [('calf_l','r_calf'),('calf_r','l_calf')]:
    c=links[old].find('collision'); o=c.find('origin')
    A=world(old)@H(rpy2R(*fl(o.get('rpy'))),fl(o.get('xyz')))
    tris=[(A[:3,:3]@t.T).T+A[:3,3] for t in stl(f"{D}/"+c.find('geometry/mesh').get('filename').replace('package://',''))]
    B={}
    for t in tris:
        nn=np.cross(t[1]-t[0],t[2]-t[0]); ar=np.linalg.norm(nn)/2
        if ar<1e-12: continue
        nn=nn/(2*ar)
        if nn[2]<-0.5:
            k=tuple(np.round(nn,2)); B.setdefault(k,[0,np.zeros(3),np.zeros(3)])
            B[k][0]+=ar; B[k][1]+=ar*t.mean(0); B[k][2]+=ar*nn
    k,(ar,cs,ns)=max(B.items(),key=lambda kv:kv[1][0])
    cen=cs/ar; z=-ns/np.linalg.norm(ns)
    x=np.array([1,0,0])-z*z[0]; x/=np.linalg.norm(x); y=np.cross(z,x)
    Tw=H(np.column_stack([x,y,z]),cen)
    Tl=np.linalg.inv(world(old))@Tw
    SOLE[new]=(Tl,cen,ar)
    print(f"{new}: sole center(world,zero pose)={np.round(cen*1000,1)} mm, area={ar*1e6:.0f} mm2")
for _n in list(links): world(_n)   # cache world poses before renaming
# rename links
for l in r.findall('link'):
    if l.get('name') in RENAME: l.set('name',RENAME[l.get('name')])
for j in joints:
    for tag in ('parent','child'):
        e=j.find(tag); e.set('link',RENAME.get(e.get('link'),e.get('link')))
# axes + limits
for j in joints:
    n=j.get('name'); old_child=[k for k,v in RENAME.items() if v==j.find('child').get('link')][0]
    a=world(old_child)[:3,2]
    if np.dot(a,WANT_AXIS[n])<0: j.find('axis').set('xyz','0 0 -1'); flipped=True
    else: j.find('axis').set('xyz','0 0 1'); flipped=False
    lo,hi=LIM[n]; ef,ve=ACT[JACT[n]]
    lim=j.find('limit'); lim.set('lower',f'{lo*DEG:.6f}'); lim.set('upper',f'{hi*DEG:.6f}')
    lim.set('effort',str(ef)); lim.set('velocity',str(ve))
    print(f"{n:13s} world_axis_before={np.round(a,2)} flipped={flipped} limit=[{lo},{hi}]deg {JACT[n]}")
# sole frames
for new,(Tl,_,_) in SOLE.items():
    side=new[0]
    link=ET.SubElement(r,'link',{'name':f'{side}_sole'})
    j=ET.SubElement(r,'joint',{'name':f'{side}_sole_fixed','type':'fixed'})
    rpy=R2rpy(Tl[:3,:3]); p=Tl[:3,3]
    ET.SubElement(j,'origin',{'xyz':f'{p[0]:.6g} {p[1]:.6g} {p[2]:.6g}','rpy':f'{rpy[0]:.6g} {rpy[1]:.6g} {rpy[2]:.6g}'})
    ET.SubElement(j,'parent',{'link':new}); ET.SubElement(j,'child',{'link':f'{side}_sole'})
r.set('name','tilt')
r.insert(0,ET.Comment(' post-processed by fix_tilt_model.py: links renamed, joint signs unified (tilt_config.h), limits added (arm/head = placeholder), sole frames added. effort/velocity approximate. '))
ET.indent(tree,'  '); tree.write(f'{D}/robot.urdf',encoding='utf-8',xml_declaration=True)

# ---------- MJCF ----------
rawm=f'{M}/robot_raw.xml'
if not os.path.exists(rawm): shutil.copy(f'{M}/robot.xml',rawm)
mt=parse(rawm); mr=mt.getroot()
flipset={j.get('name') for j in joints if j.find('axis').get('xyz')=='0 0 -1'}
for b in mr.iter('body'):
    if b.get('name') in RENAME: b.set('name',RENAME[b.get('name')])
for jt in mr.iter('joint'):
    n=jt.get('name')
    if n in LIM:
        jt.set('axis','0 0 -1' if n in flipset else '0 0 1')
        jt.set('range',f'{LIM[n][0]*DEG:.6f} {LIM[n][1]*DEG:.6f}')
for fj in mr.iter('freejoint'):
    fj.set('name','torso_freejoint')
for a in mr.iter('position'):
    n=a.get('joint')
    if n in LIM:
        a.set('ctrlrange',f'{LIM[n][0]*DEG:.6f} {LIM[n][1]*DEG:.6f}')
        ef=ACT[JACT[n]][0]; a.set('forcerange',f'{-ef} {ef}')
for b in mr.iter('body'):
    if b.get('name') in SOLE:
        Tl=SOLE[b.get('name')][0]; q=R2quat(Tl[:3,:3]); p=Tl[:3,3]
        ET.SubElement(b,'site',{'name':f"{b.get('name')[0]}_sole",'pos':f'{p[0]:.6g} {p[1]:.6g} {p[2]:.6g}','quat':' '.join(f'{v:.6g}' for v in q),'size':'0.005'})
ET.indent(mt,'  '); mt.write(f'{M}/robot.xml',encoding='utf-8')
# view copy (base welded in air, contacts off in scene_view.xml)
s=open(f'{M}/robot.xml').read()
import re
s=re.sub(r'<freejoint[^>]*/>','<!-- freejoint removed for viewing -->',s,1)
s=s.replace('<body name="torso" pos="0 0 0"','<body name="torso" pos="0 0 0.3"',1)
open(f'{M}/robot_view.xml','w').write(s)
print("written: robot.urdf, robot.xml, robot_view.xml")
