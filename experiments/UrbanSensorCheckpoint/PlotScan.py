from pathlib import Path
import csv, math, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont

root=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parent/'run'
lines=(root/'urban_world.pcd').read_text().splitlines()
start=lines.index('DATA ascii')+1
data=np.array([[float(v) for v in row.split()] for row in lines[start:] if row.strip()])
x,y,z=data[:,0],data[:,1],data[:,2]
sx=x; sz=z+70; heights=y-7
rows=list(csv.DictReader((root/'pulses.csv').open()))
font='C:/Windows/Fonts/msyh.ttc'
F=lambda size:ImageFont.truetype(font,size)
big,medium,small,tiny=F(36),F(25),F(21),F(18)
ink='#172c43';muted='#54697c';grid='#dce5ed'
canvas=Image.new('RGB',(1800,1340),'#f4f7fb'); d=ImageDraw.Draw(canvas)
d.text((55,26),'Urban · HELIOS 经验材质单帧扫描',font=big,fill=ink)
d.text((58,81),'16 线 × 720 方位  |  905 nm  |  高斯光束，16 条 MC 射线/脉冲  |  固定 seed 42/43  |  无电压噪声',font=small,fill=muted)
d.text((58,114),f'{len(rows):,} 次发射 → {len(data):,} 个测距点；点来自 ADC 波形测距，不是直接导出几何交点。',font=small,fill=muted)

stops=[(0,(38,72,133)),(.25,(30,137,163)),(.5,(82,185,140)),(.75,(203,212,85)),(1,(245,178,62))]
def color(value):
    t=float(np.clip(value,0,1))
    for (a,ca),(b,cb) in zip(stops[:-1],stops[1:]):
        if t<=b:
            u=(t-a)/(b-a); return tuple(round(ca[i]+u*(cb[i]-ca[i])) for i in range(3))
    return stops[-1][1]
def card(box,title,subtitle):
    d.rounded_rectangle(box,radius=18,fill='white',outline=grid,width=1)
    d.text((box[0]+24,box[1]+18),title,font=medium,fill=ink)
    d.text((box[0]+24,box[1]+57),subtitle,font=tiny,fill=muted)

card((45,164,885,865),'斜视点云','颜色表示世界高度 Y − 7 m；传感器处路面 Y = 7 m')
card((915,164,1755,865),'俯视点云','以传感器为原点：横轴 ΔX，纵轴 ΔZ，单位 m')

# Orthographic oblique projection of actual measured points, not a scene render.
u=.82*sx-.57*sz; v=.25*sx+.36*sz+1.8*heights
umin,umax=float(u.min())-10,float(u.max())+10
vmin,vmax=float(v.min())-10,float(v.max())+10
scale=min(770/(umax-umin),530/(vmax-vmin))
uc=(umin+umax)/2;vc=(vmin+vmax)/2
def iso(px,py,pz):
    pu=.82*px-.57*pz;pv=.25*px+.36*pz+1.8*py
    return 465+(pu-uc)*scale,550-(pv-vc)*scale
iso_layer=canvas.copy(); d=ImageDraw.Draw(iso_layer)
for k in range(-100,101,25):
    d.line([iso(-100,0,k),iso(100,0,k)],fill='#e9eef3',width=1)
    d.line([iso(k,0,-100),iso(k,0,100)],fill='#e9eef3',width=1)
for i in np.argsort(sx+sz):
    px,py=iso(sx[i],heights[i],sz[i]); c=color(heights[i]/20)
    d.ellipse((px-1.5,py-1.5,px+1.5,py+1.5),fill=c)
sensor=iso(0,1.8,0)
d.ellipse((sensor[0]-5,sensor[1]-5,sensor[0]+5,sensor[1]+5),fill='#d64c51',outline='white',width=1)
d.text((sensor[0]+9,sensor[1]-22),'传感器',font=tiny,fill='#b43840')
for vx,vy,vz,label in [(20,0,0,'X'),(0,0,20,'Z'),(0,10,0,'Y')]:
    a=iso(-75,0,-75);b=iso(-75+vx,vy,-75+vz)
    d.line([a,b],fill=ink,width=2);d.text((b[0]+3,b[1]-18),label,font=tiny,fill=ink)
canvas.paste(iso_layer.crop((48,265,882,802)),(48,265)); d=ImageDraw.Draw(canvas)

# Full horizontal extent around sensor; no hidden crop.
left,right,top,bottom=1005,1685,290,800
limit=120; sc=min((right-left)/(2*limit),(bottom-top)/(2*limit))
cx,cz=(left+right)/2,(top+bottom)/2
def topxy(px,pz):return cx+px*sc,cz-pz*sc
for val in range(-100,101,50):
    xx,_=topxy(val,0);_,zz=topxy(0,val)
    d.line((xx,top,xx,bottom),fill=grid);d.line((left,zz,right,zz),fill=grid)
    d.text((xx-18,bottom+8),str(val),font=tiny,fill=muted)
    d.text((left-43,zz-12),str(val),font=tiny,fill=muted)
for i in range(len(data)):
    px,py=topxy(sx[i],sz[i]);d.ellipse((px-1.25,py-1.25,px+1.25,py+1.25),fill=color(heights[i]/20))
d.ellipse((cx-5,cz-5,cx+5,cz+5),fill='#d64c51',outline='white',width=1)
d.text((1645,823),'ΔX (m)',font=tiny,fill=muted);d.text((939,285),'ΔZ',font=tiny,fill=muted)

# Shared height legend in first card.
for k in range(240):d.line((75+k,815,75+k,830),fill=color(k/239))
d.text((75,835),'0 m',font=tiny,fill=muted);d.text((270,835),'20 m+',font=tiny,fill=muted)

card((45,890,1755,1267),'旋转扫描距离图','横轴是方位角；每行是一条扫描线。浅灰色表示未输出测距点。颜色表示测得距离，不是反射率。')
heat=Image.new('RGB',(720,16),'#e7edf3'); hp=heat.load()
for r in rows:
    if r['detected']=='1':
        column=round(float(r['azimuth_deg'])/.5);ring=int(r['ring'])
        hp[column,15-ring]=color(float(r['range_m'])/120)
heat=heat.resize((1440,224),Image.Resampling.NEAREST)
canvas.paste(heat,(135,992))
for azi in range(0,361,60):
    px=135+azi/360*1440;d.text((px-15,1220),f'{azi}°',font=tiny,fill=muted)
for val in [15,7,-1,-9,-17,-25]:
    row=(15-val)/40*15;py=992+(row+.5)*14
    d.text((65,py-10),f'{val}°',font=tiny,fill=muted)
for k in range(224):d.line((1620,992+k,1642,992+k),fill=color(1-k/223))
d.text((1650,984),'120 m',font=tiny,fill=muted);d.text((1650,1195),'0 m',font=tiny,fill=muted)
d.text((57,1290),'静态场景、固定平台；几何约 244.5 万三角形。未模拟大气衰减、接收器噪声、植被透射或多次散射。',font=small,fill=muted)
canvas.save(root/'urban_scan.png')

# Independent arithmetic/serialization checks on the saved files.
assert len(rows)==11520 and len(data)>0
assert sum(int(r['detected']) for r in rows)==len(data)
assert np.isfinite(data).all()
assert (data[:,6]>=0).all()
sensor=(root/'urban_sensor.pcd').read_text().splitlines();i=sensor.index('DATA ascii')+1
sp=np.array([[float(v) for v in row.split()] for row in sensor[i:] if row.strip()])
assert np.allclose(np.sqrt((sp[:,:3]**2).sum(axis=1)),sp[:,6],atol=2e-5,rtol=1e-6)
print(f'PCD round-trip: {len(data)} finite points, sensor-coordinate norms match measured ranges.')
print('Saved:',root/'urban_scan.png')
