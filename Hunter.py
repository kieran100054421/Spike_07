"""
An Hunter with wander behaviour

Created by Kieran Bates
"""

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange, uniform

class Hunter(object):

    def __init__(self, world=None, scale=30.0, mass=1.0, mode='wander', visible=False):
        self.world = world
        self.mode = mode
        self.visible = visible
        dir = radians(random() * 360)
        self.pos = Vector2D(randrange(world.cx), randrange(world.cy))
        self.vel = Vector2D()
        self.heading = Vector2D(sin(dir), cos(dir))
        self.side = self.heading.perp()
        self.scale = Vector2D(scale, scale)
        self.acceleration = Vector2D()
        self.mass = mass

        # wander info
        self.wander_target = Vector2D(1, 0)
        self.wander_dist = 1.0 * scale
        self.wander_radius = 1.0 * scale
        self.wander_jitter = 10.0 * scale
        self.bRadius = scale

        # limits
        self.max_speed = 20.0 * scale
        self.max_force = 500.0

        # data for drawing hunter
        self.color = 'BLUE'
        self.vehicle_shape = [
            Point2D(-1.0, 0.6),
            Point2D(1.0, 0.0),
            Point2D(-1.0, -0.6)
        ]

    def set_visible(self, visible):
        """sets whether the hunter is visible"""
        self.visible = visible

    def calculate(self, delta):
        """resets the current steering force according to mode"""
        mode = self.mode

        if mode == 'wander':
            accel = self.wander(delta)
        else:
            accel = Vector2D()

        self.acceleration = accel

        return accel

    def update(self, delta):
        """update vehicle position and orientation"""
        if self.visible:
            acceleration = self.calculate(delta)
            # new velocity
            self.vel += acceleration * delta
            # check for limits of the new velocity
            self.vel.truncate(self.max_speed)
            # update position
            self.pos += self.vel * delta
            # update heading is non-zero velocity (moving)
            if self.vel.length_sq() > 0.00000001:
                self.heading = self.vel.get_normalised()
                self.side = self.heading.perp()

            # treat world as continuous space
            self.world.wrap_around(self.pos)
            force = self.calculate(delta)
            force.truncate(self.max_force)

    def render(self, color=None):
        """Displays the Hunter on the screen"""
        if self.visible:
            egi.set_pen_color(name=self.color)
            pts = self.world.transform_points(self.vehicle_shape, self.pos, self.heading, self.side, self.scale)

            # draw wander info
            # if self.mode == 'wander':
            #     # calculate the centre of the wander circle in front of the agent
            #     wnd_pos = Vector2D(self.wander_dist, 0)
            #     wnd_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
            #     # draw wander circle
            #     egi.green_pen()
            #     egi.circle(wnd_pos, self.wander_radius)
            #     # draw the target (little circle on the big circle)
            #     egi.red_pen()
            #     wnd_pos = (self.wander_target + Vector2D(self.wander_dist, 0))
            #     wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
            #     egi.circle(wld_pos, 3)

            # draw it
            egi.closed_shape(pts)

    def speed(self):
        """gets the hunter's speed"""
        return self.vel.length()

    # ------------------------------------------------------------------

    def seek(self, target_pos):
        """moves towards the target position"""
        desired_vel = (target_pos - self.pos).normalise() * self.max_speed

        return desired_vel - self.vel

    def wander(self, delta):
        """random wandering using projected jitter sphere"""
        wt = self.wander_target
        # this behaviour is dependant on the update rate, so this line must
        # be included when using time independant frame rate
        jitter_tts = self.wander_jitter * delta
        # first, add a small random vector to the targets position
        wt += Vector2D(uniform(-1, 1) * jitter_tts, uniform(-1, 1) * jitter_tts)
        # re-project this new vector back on to a unit circle
        wt.normalise()
        # increase the length of the vector to the same radius
        # of the wander circle
        wt *= self.wander_radius
        # move the target into a position WonderDist in front of the agent
        target = wt + Vector2D(self.wander_dist, 0)
        # project the target into world space
        wld_target = self.world.transform_point(target, self.pos, self.heading, self.side)

        return self.seek(wld_target)


