from pathlib import Path
import csv, json, math, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont

root=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parent/'run'
meta=json.loads((root/'camera_rig.json').read_text(encoding='utf-8'))
rgb=Image.open(root/'urban_rgb.ppm').convert('RGB')
assert rgb.size==(meta['width'],meta['height'])
assert meta['camera_to_lidar_translation_m']==[0,0,0]
assert meta['camera_to_lidar_orientation_wxyz']==[1,0,0,0]
rgb.save(root/'urban_rgb.png')
overlay=rgb.convert('RGBA'); layer=Image.new('RGBA',rgb.size,(0,0,0,0));od=ImageDraw.Draw(layer)
rows=list(csv.DictReader((root/'lidar_projection.csv').open()))
# Independent check: recover camera coordinates and projection from saved PCD + K.
source=Path(meta['source_lidar_pcd']).read_text().splitlines()
view=[float(v) for v in next(s for s in source if s.startswith('VIEWPOINT ')).split()[1:]]
assert np.allclose(view[:3],meta['rig_origin_world_m'],atol=1e-7,rtol=0)
assert np.allclose(view[3:],meta['rig_orientation_wxyz'],atol=1e-7,rtol=0)
first=source.index('DATA ascii')+1
points=np.array([[float(v) for v in s.split()[:3]] for s in source[first:] if s.strip()])
w,x,y,z=meta['rig_orientation_wxyz']
rotation=np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                   [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                   [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
local=(points-np.array(meta['rig_origin_world_m']))@rotation
max_projection_difference=0.0
for r in rows:
    if r['in_image']=='1':
        p=local[int(r['point_index'])]
        u=meta['cx']+meta['fx']*p[0]/p[2];v=meta['cy']-meta['fy']*p[1]/p[2]
        max_projection_difference=max(max_projection_difference,abs(u-float(r['pixel_x'])),abs(v-float(r['pixel_y'])))
assert max_projection_difference<1e-3
with (root/'urban_rgb_linear.pfm').open('rb') as f:
    assert f.readline().strip()==b'PF'
    assert f.readline().strip()==b'640 360'
    assert f.readline().strip()==b'-1.0'
    linear=np.frombuffer(f.read(),dtype='<f4')
assert linear.size==640*360*3 and np.isfinite(linear).all() and (linear>=0).all()
visible=[r for r in rows if r['in_image']=='1' and r['center_geometry_consistent']=='1']
def col(t):
    t=max(0,min(1,t))
    a=(33,182,235);b=(255,180,40)
    return tuple(round(x+(y-x)*t) for x,y in zip(a,b))
for r in sorted(visible,key=lambda r:float(r['camera_z_m']),reverse=True):
    x,y=float(r['pixel_x']),float(r['pixel_y']);z=float(r['camera_z_m'])
    od.ellipse((x-1.4,y-1.4,x+1.4,y+1.4),fill=col(z/120)+(215,))
overlay=Image.alpha_composite(overlay,layer).convert('RGB');overlay.save(root/'urban_rgb_lidar_overlay.png')
font='C:/Windows/Fonts/msyh.ttc'
large=ImageFont.truetype(font,30);medium=ImageFont.truetype(font,23);small=ImageFont.truetype(font,19)
canvas=Image.new('RGB',(1400,590),'#f4f7fb');d=ImageDraw.Draw(canvas)
ink='#172c43';muted='#52697e'
d.text((34,20),'同位姿 RGB 相机与 LiDAR · Urban',font=large,fill=ink)
d.text((35,63),'共享位姿：(0, 8.8, −70) m  |  前向 +Z，上向 +Y  |  相机相对雷达：零平移、零旋转',font=small,fill=muted)
d.text((38,107),'RGB 路径追踪预览',font=medium,fill=ink)
d.text((722,107),f'测距点投影叠加 · {len(visible):,} 点',font=medium,fill=ink)
canvas.paste(rgb,(36,145));canvas.paste(overlay,(724,145))
d.text((36,521),'640 × 360 · 16 spp · 水平视场 90° · MTL Kd 漫反射',font=small,fill=muted)
d.text((724,521),'点颜色表示相机 Z 深度；只展示前向视野中的点',font=small,fill=muted)
d.text((36,557),'静态场景与静止平台；不是完整材质复现。树木源材质未提供 RGB 颜色，因此保留为黑色。',font=small,fill=muted)
canvas.save(root/'rgb_lidar_comparison.png')
print('Saved clean RGB, projected-point overlay and comparison figure.')
print('in-view points=',sum(r['in_image']=='1' for r in rows),'overlay points=',len(visible))
print('vertical FOV degrees=',math.degrees(2*math.atan(meta['height']/(2*meta['fy']))))
print('Independent projection max pixel difference=',max_projection_difference)
print('Linear PFM verified: finite, nonnegative, correct dimensions.')
