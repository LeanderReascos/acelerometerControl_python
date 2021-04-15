import vpython as vp
import numpy as np       


sky = []
walls = []
w = 100
l = w*1.7
N = 3
for i in range(N):
    for j in range(N):
        b = vp.box(pos = vp.vector(-w+w*i,-l+l*j,100),height=1,width=l,length=w,up=vp.vector(0,0,1))
        b.texture =  'Resources\\sky.jpg'
        sky.append(b)


N = 3
names = ['wall1.png','wall2.png','wall3.png']
for i in range(2):
    for j in range(N):
        b = vp.box(pos = vp.vector(-3*w/2+w*3*i,-l+l*j,50),height=w,width=l,length=0.5,up=vp.vector(0,0,1))
        b.texture =  'Resources\\'+names[j]
        walls.append(b)

for i in range(N):
    for j in range(2):
        b = vp.box(pos = vp.vector(-w+w*i,-3*l/2+l*3*j,50),height=w,width=0.5,length=w,up=vp.vector(0,0,1))
        b.texture =  'Resources\\sky.jpg'
        walls.append(b)