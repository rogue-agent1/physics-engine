#!/usr/bin/env python3
"""Simple 2D physics engine — zero deps."""
import math
class Vec2:
    def __init__(self,x=0,y=0):self.x=x;self.y=y
    def __add__(self,o):return Vec2(self.x+o.x,self.y+o.y)
    def __sub__(self,o):return Vec2(self.x-o.x,self.y-o.y)
    def __mul__(self,s):return Vec2(self.x*s,self.y*s)
    def dot(self,o):return self.x*o.x+self.y*o.y
    def length(self):return math.sqrt(self.x**2+self.y**2)
    def normalize(self):l=self.length();return Vec2(self.x/l,self.y/l) if l>0 else Vec2()

class Body:
    def __init__(self,pos,vel=None,mass=1,radius=1):
        self.pos=pos;self.vel=vel or Vec2();self.mass=mass;self.radius=radius;self.force=Vec2()
    def apply_force(self,f):self.force=self.force+f

class World:
    def __init__(self,gravity=Vec2(0,-9.81)):self.bodies=[];self.gravity=gravity
    def add(self,body):self.bodies.append(body)
    def step(self,dt):
        for b in self.bodies:
            b.apply_force(self.gravity*b.mass)
            acc=Vec2(b.force.x/b.mass,b.force.y/b.mass)
            b.vel=b.vel+acc*dt;b.pos=b.pos+b.vel*dt;b.force=Vec2()
        # Collision detection
        for i in range(len(self.bodies)):
            for j in range(i+1,len(self.bodies)):
                a,b=self.bodies[i],self.bodies[j]
                d=a.pos-b.pos;dist=d.length()
                if dist<a.radius+b.radius and dist>0:
                    n=d.normalize()
                    rv=a.vel-b.vel;vn=rv.dot(n)
                    if vn>0:continue
                    imp=Vec2(n.x*(-vn),n.y*(-vn))
                    a.vel=a.vel+imp*(1/a.mass);b.vel=b.vel-imp*(1/b.mass)
def test():
    w=World();b=Body(Vec2(0,10))
    w.add(b)
    for _ in range(100):w.step(0.01)
    assert b.pos.y<10  # fell due to gravity
    assert b.vel.y<0
    print(f"After 1s: pos=({b.pos.x:.1f},{b.pos.y:.1f}), vel=({b.vel.x:.1f},{b.vel.y:.1f})")
    print("All tests passed!")
if __name__=="__main__":test()
