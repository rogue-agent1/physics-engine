#!/usr/bin/env python3
"""physics_engine - 2D physics engine with rigid bodies and collision."""
import sys, math

class Vec2:
    def __init__(self, x=0, y=0): self.x = x; self.y = y
    def __add__(self, o): return Vec2(self.x+o.x, self.y+o.y)
    def __sub__(self, o): return Vec2(self.x-o.x, self.y-o.y)
    def __mul__(self, s): return Vec2(self.x*s, self.y*s)
    def dot(self, o): return self.x*o.x + self.y*o.y
    def length(self): return math.sqrt(self.x**2 + self.y**2)
    def normalize(self):
        l = self.length()
        return Vec2(self.x/l, self.y/l) if l > 0 else Vec2()

class Body:
    def __init__(self, x, y, mass=1, radius=1, static=False):
        self.pos = Vec2(x, y)
        self.vel = Vec2()
        self.acc = Vec2()
        self.mass = mass
        self.radius = radius
        self.static = static
        self.restitution = 0.8
    def apply_force(self, fx, fy):
        if not self.static:
            self.acc = self.acc + Vec2(fx/self.mass, fy/self.mass)
    def update(self, dt):
        if self.static: return
        self.vel = self.vel + self.acc * dt
        self.pos = self.pos + self.vel * dt
        self.acc = Vec2()

class World:
    def __init__(self, gravity=Vec2(0, -9.81)):
        self.bodies = []
        self.gravity = gravity
    def add(self, body):
        self.bodies.append(body)
        return body
    def step(self, dt):
        for b in self.bodies:
            if not b.static:
                b.apply_force(self.gravity.x * b.mass, self.gravity.y * b.mass)
            b.update(dt)
        self._resolve_collisions()
    def _resolve_collisions(self):
        for i in range(len(self.bodies)):
            for j in range(i+1, len(self.bodies)):
                a, b = self.bodies[i], self.bodies[j]
                d = b.pos - a.pos
                dist = d.length()
                if dist < a.radius + b.radius and dist > 0:
                    n = d.normalize()
                    overlap = a.radius + b.radius - dist
                    if not a.static and not b.static:
                        a.pos = a.pos - n * (overlap/2)
                        b.pos = b.pos + n * (overlap/2)
                    elif a.static:
                        b.pos = b.pos + n * overlap
                    else:
                        a.pos = a.pos - n * overlap
                    rv = b.vel - a.vel
                    vn = rv.dot(n)
                    if vn > 0: continue
                    e = min(a.restitution, b.restitution)
                    j_mag = -(1+e) * vn
                    inv_a = 0 if a.static else 1/a.mass
                    inv_b = 0 if b.static else 1/b.mass
                    j_mag /= (inv_a + inv_b)
                    impulse = n * j_mag
                    if not a.static: a.vel = a.vel - impulse * (1/a.mass)
                    if not b.static: b.vel = b.vel + impulse * (1/b.mass)

def test():
    w = World()
    ball = w.add(Body(0, 10, mass=1, radius=0.5))
    floor = w.add(Body(0, 0, mass=1, radius=100, static=True))
    for _ in range(1000):
        w.step(0.01)
    assert ball.pos.y > -1  # ball should have bounced, not fallen through
    assert abs(ball.vel.y) < 5  # energy lost to restitution
    # two balls colliding
    w2 = World(gravity=Vec2(0, 0))
    a = w2.add(Body(0, 0, mass=1, radius=1))
    b = w2.add(Body(5, 0, mass=1, radius=1))
    a.vel = Vec2(2, 0)
    for _ in range(200):
        w2.step(0.01)
    assert b.vel.x > 0  # b got pushed
    print("OK: physics_engine")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        test()
    else:
        print("Usage: physics_engine.py test")
