#!/usr/bin/env python

from calendar import leapdays
from math import dist
import numpy as np
from sklearn import mixture
from pycrazyswarm import *
import time

np.seterr(divide='ignore', invalid='ignore')
Z = 1.0
sleepRate = 60


class boid:
    def __init__(self,cf,role):
        self.cf = cf
        self.role = role
        self.state = "avoid"


def closest_leader(boid,leaders):
    closest = None
    dist = 0
    for leader in leaders:
        if leader is not boid:
            d = np.linalg.norm(leader.cf.position() - boid.cf.position())
            if closest is None or d < dist:
                closest = leader
                dist = d
    return closest

# boid simulation following a leader
def following(boids,leaders):
    for b in boids:
        if b.role == "leader":
            if b.state == "avoid":
                closest = closest_leader(b,leaders)
                if closest is not None:
                    dist = np.linalg.norm(b.cf.position() - closest.cf.position())
                    if dist <10000:
                        force = 3*((b.cf.position() - closest.cf.position()) / (dist**4)) + (b.cf.velocity() + closest.cf.velocity())*1.1
                        vel = (2*b.cf.velocity() + force)/3
                        f = np.linalg.norm(vel)
                        if f > 0.25:
                            vel = 0.25 * vel / f
                        b.cf.cmdVelocityWorld(vel, yawRate=0)
            elif b.state == "fix":
                 b.cf.cmdVelocityWorld([0,0,0], yawRate=0)
            elif b.state == "circle":
                center_circle = np.array([2.0, 2.0, 2.0])
                radius = 0.3
                dist = np.linalg.norm(b.cf.position() - center_circle)
                if dist < radius:
                    force = (-center_circle + b.cf.position()) / (dist**2)
                else:
                    force = (center_circle - b.cf.position()) / (dist**2)
                tt = time.time()/10000
                force += np.array([np.cos(tt), np.sin(tt), np.cos(tt)])
                vel = (2*b.cf.velocity() + force)/3
                f = np.linalg.norm(vel)
                if f > 0.25:
                    vel = 0.25 * vel / f
                b.cf.cmdVelocityWorld(vel, yawRate=0)
        else:
            closest = closest_leader(b,leaders)
            if closest is not None:
                # attracted to closest leader
                force = (closest.cf.position() - b.cf.position()) / np.linalg.norm(closest.cf.position() - b.cf.position())**2
                # repelled from other boids
                for other in boids:
                    if other is not b:
                        dist = np.linalg.norm(b.cf.position() - other.cf.position())
                        if dist < 0.75:
                            force += ((b.cf.position() - other.cf.position()) / (dist**2))
                        if dist < 2: # alignment
                            force += (other.cf.velocity() + b.cf.velocity())*1.1
                vel = (2*b.cf.velocity() + force)/3
                f = np.linalg.norm(vel)
                if f > 0.25:
                    vel = 0.25 * vel / f
                b.cf.cmdVelocityWorld(vel, yawRate=0)
        if b.cf.position()[0] < -4:
            b.cf.cmdVelocityWorld([0.5,0,0], yawRate=0)
        if b.cf.position()[0] > 4:
            b.cf.cmdVelocityWorld([-0.5,0,0], yawRate=0)
        if b.cf.position()[1] < -4:
            b.cf.cmdVelocityWorld([0,0.5,0], yawRate=0)
        if b.cf.position()[1] > 4:
            b.cf.cmdVelocityWorld([0,-0.5,0], yawRate=0)
        if b.cf.position()[2] < 0:
            b.cf.cmdVelocityWorld([0,0,0.5], yawRate=0)
        if b.cf.position()[2] > 4:
            b.cf.cmdVelocityWorld([0,0,-0.5], yawRate=0)


# boid simulation repelling from other boids
def repelling(boids,leaders):
    for b in boids:
        force = np.array([0.0, 0.0, 0.0])
        for other in boids:
            if other is not b:
                dist = np.linalg.norm(b.position() - other.position())
                if dist < 1: # repulsion
                    force += (b.position() - other.position()) / (dist**2)
                elif (dist < 4) and (other in leaders) and (b not in leaders): # attraction
                    force += ((other.position() - b.position()) / (dist))/10
                # if (dist < 4) and (other in leaders and (b not in leaders)):
                #     force += ((other.position() - b.position()) / (dist))
                # if dist < 1: # alignment
                #     force += ((other.velocity() - b.velocity()) / np.linalg.norm(other.velocity() - b.velocity()))/100
        if b in leaders:
            for other in leaders:
                if other is not b:
                    force += 100*(b.position() - other.position()) / (dist**4)
        # box
        if b.position()[0] > 4:
            force[0] -= 2
        if b.position()[0] < -4:
            force[0] += 2
        if b.position()[1] > 4:
            force[1] -= 2
        if b.position()[1] < -4:
            force[1] += 2
        if b.position()[2] > 4:
            force[2] -= 2
        if b.position()[2] < -4:
            force[2] += 2
        # normalize the velocity
        vel = (2*b.velocity() + force)/3
        f = np.linalg.norm(vel)
        if f > 0.25:
            vel = 0.25 * vel / f
        b.cmdVelocityWorld(vel, yawRate=0)





if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    print(f"found a total of {len(allcfs.crazyflies)} boids")
    # allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1)

    boids = []
    leaders = []

    # random velocity
    for cf in allcfs.crazyflies:
        cf.cmdVelocityWorld(np.random.rand(3)*10,yawRate=0)
        boids.append(boid(cf,"flock"))

    # leader
    # choose a random leader
    for _ in range(len(boids)//10):
        i = np.random.randint(len(boids))
        boids[i].role = "leader"
        boids[i].cf.setLEDColor(1.0,0.0,0.0)
        leaders.append(boids[i])
        # boids.pop(i)
    
    leaders[0].state = "fix"
    leaders[0].cf.setLEDColor(0.0,1.0,0.0)


    # run sim for 1min
    while True:
        following(boids,leaders)
        # repelling(allcfs.crazyflies,leaders)
        timeHelper.sleepForRate(sleepRate)