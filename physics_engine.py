#!/usr/bin/env python3
"""physics_engine - 2D rigid body physics."""
import argparse, math, sys, time

class Vec2:
    def __init__(self, x=0, y=0): self.x=x; self.y=y
    def __add__(self, o): return Vec2(self.x+o.x, self.y+o.y)
    def __sub__(self, o): return Vec2(self.x-o.x, self.y-o.y)
    def __mul__(self, s): return Vec2(self.x*s, self.y*s)
    def dot(self, o): return self.x*o.x + self.y*o.y
    def length(self): return math.sqrt(self.x**2+self.y**2)
    def normalize(self): l=self.length(); return Vec2(self.x/l,self.y/l) if l>0 else Vec2()

class Body:
    def __init__(self, x, y, r, mass=1, vx=0, vy=0):
        self.pos=Vec2(x,y); self.vel=Vec2(vx,vy); self.r=r; self.mass=mass
        self.restitution=0.8

class World:
    def __init__(self, w=80, h=40, gravity=9.8):
        self.w=w; self.h=h; self.gravity=gravity; self.bodies=[]
    def add(self, body): self.bodies.append(body); return body
    def step(self, dt=0.016):
        for b in self.bodies:
            b.vel.y += self.gravity * dt
            b.pos = b.pos + b.vel * dt
        # Wall collisions
        for b in self.bodies:
            if b.pos.y + b.r > self.h: b.pos.y = self.h - b.r; b.vel.y *= -b.restitution
            if b.pos.y - b.r < 0: b.pos.y = b.r; b.vel.y *= -b.restitution
            if b.pos.x + b.r > self.w: b.pos.x = self.w - b.r; b.vel.x *= -b.restitution
            if b.pos.x - b.r < 0: b.pos.x = b.r; b.vel.x *= -b.restitution
        # Body-body collisions
        for i in range(len(self.bodies)):
            for j in range(i+1, len(self.bodies)):
                a, b = self.bodies[i], self.bodies[j]
                d = a.pos - b.pos; dist = d.length()
                if dist < a.r + b.r and dist > 0:
                    n = d.normalize(); overlap = a.r + b.r - dist
                    a.pos = a.pos + n * (overlap/2); b.pos = b.pos - n * (overlap/2)
                    rv = a.vel - b.vel; vn = rv.dot(n)
                    if vn > 0: continue
                    e = min(a.restitution, b.restitution)
                    j_imp = -(1+e)*vn / (1/a.mass + 1/b.mass)
                    a.vel = a.vel + n * (j_imp/a.mass)
                    b.vel = b.vel - n * (j_imp/b.mass)
    def render(self):
        grid = [["."]*self.w for _ in range(self.h)]
        for i, b in enumerate(self.bodies):
            x, y = int(b.pos.x), int(b.pos.y)
            for dy in range(-int(b.r), int(b.r)+1):
                for dx in range(-int(b.r), int(b.r)+1):
                    if dx*dx+dy*dy <= b.r*b.r:
                        px, py = x+dx, y+dy
                        if 0<=px<self.w and 0<=py<self.h: grid[py][px] = str(i)
        return "\n".join("".join(row) for row in grid)

def main():
    p = argparse.ArgumentParser(description="2D physics engine")
    p.add_argument("-s","--steps", type=int, default=60)
    a = p.parse_args()
    w = World(60, 30)
    w.add(Body(10, 5, 2, vx=8, vy=0))
    w.add(Body(30, 3, 3, vx=-3, vy=2))
    w.add(Body(50, 10, 2, vx=-5, vy=-3))
    for s in range(a.steps):
        sys.stdout.write(f"\033[2J\033[HStep {s} | Bodies: {len(w.bodies)}\n")
        print(w.render())
        w.step(0.15)
        time.sleep(0.05)

if __name__ == "__main__": main()
