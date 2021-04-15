from vpython import *
import numpy as np
from pygame import mixer
mixer.init()
mixer.music.load("Resources\\laser.mp3")

def make_hemisphere(r):
	circle = shapes.circle(radius=r,angle1=0,angle2=pi) #you can use the size of your choice
	circular_path = []
	ang = 0
	while ang<=pi:
		circular_path.append(vector(cos(ang)*0.01,sin(ang)*0.01,0))
		ang += pi/100
	return extrusion(path=circular_path,shape=circle)

class BB8(object):
	"""docstring for BB8"""
	def __init__(self,vmax = 0.01):
		d = 0.1
		self.__body = sphere(radius=2,texture='Resources\\bb8.jpg')
		self.__head  = make_hemisphere(1.2)
		self.__head.pos = vector(0,0,2.5+d)
		self.__head.up = vector(0,0,1)
		self.__dir = vector(0,0,0)
		self.__projectiles = []
		point = make_hemisphere(0.05)
		eye = make_hemisphere(0.2)
		point.pos = vector(1,0.4,2.5+d)
		eye.pos = vector(1.05,0,2.5+d)
		eye.color = color.black
		point.color = color.black
		eye.up = vector(1,0,pi/8)
		point.up = vector(1,0,pi/8)

		self.__head.texture  = 'Resources\\head.jpg'
		self.__head_atributes = compound([eye,point])

		self.__bb8 = [self.__head,self.__head_atributes]
		self.__vmax = vmax
		self.__posReference = -self.__head.pos+self.__head_atributes.pos

		self.__body.pos = vector(0,0,2)
		self.orientation(vector(0,0,1))

	def move(self,v,dt):
		self.__dir = v
		self.__body.rotate(axis=vector(-v.y,v.x,0),angle=self.__vmax*mag(v)*2*dt)
		self.__body.pos = self.__body.pos + v*self.__vmax

	def orientation(self,v):
		self.__head.up = v
		self.__head.pos = self.__body.pos + v*(2.6)
		self.__head_atributes.pos = self.__head.pos + self.__posReference

	def rotate_head(self,ang):
		self.__head.rotate(axis=vector(0,0,1),angle=ang)
		self.__head_atributes.pos = self.__head.pos + self.__posReference

	def get_pos(self):
		return vector(np.round(self.__body.pos.x,2),np.round(self.__body.pos.y,2),0)

	def move_projectiles(self,dt):
		for p in self.__projectiles:
			p.pos(dt)

	def shot(self):
		mixer.music.play()
		if len(self.__projectiles) > 0:
			for p in self.__projectiles:
				p.delete()
			self.__projectiles = []
		position = self.__head.pos
		self.__projectiles.append(Projectil(position,self.__dir))
		
			



class Projectil(object):
	"""docstring for Projectil"""
	def __init__(self, pos, dir):
		self.__p = sphere(radius=0.1,color=color.red,make_trail=True, retain=20)
		self.__p.pos = pos
		self.__p.mass = 1
		self.__p.p = dir*100

	def pos(self,dt):
		self.__p.pos += self.__p.p/self.__p.mass*dt

	def delete(self):
		self.__p.visible = False
		self.__p.clear_trail()
		del self.__p
	def get_pos(self):
		return self.__p.pos

