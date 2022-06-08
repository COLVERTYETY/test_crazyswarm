#!/usr/bin/env python

from math import dist
import numpy as np
from pycrazyswarm import *

Z = 1.0
sleepRate = 30

leaders = []

# boid simulation repelling from other boids
def repelling(boids):
    for b in boids:
        force = np.array([0, 0, 0])
        for other in boids:
            if other is not b:
                dist = np.linalg.norm(b.position() - other.position())
                if dist < 1.5:
                    force += (b.position() - other.position()) / (dist*dist)
        # normalize the velocity
        vel = b.velocity() + force
        f = np.linalg.norm(vel)
        if f > 1:
            vel = vel / f
        b.cmdVelocityWorld(vel, yawRate=0)

# boid simulation following a leader
def following(boids):
    global leaders
    for b in boids:
        force = np.array([0, 0, 0])
        for leader in leaders:
            dist = np.linalg.norm(b.position() - leader.position())
            if dist < 1:
                force += (leader.position() - b.position()) / (dist)
        # nromalize the velocity
        vel = b.velocity() + force
        f = np.linalg.norm(vel)
        if f > 1:
            vel = vel / f
        b.cmdVelocityWorld(vel, yawRate=0)

# boid simulation cohesion
def cohesion(boids):
    for b in boids:
        avgyaw = 0
        i=1
        for other in boids:
            if other is not b:
                dist = np.linalg.norm(b.position() - other.position())
                if dist < 10:
                    avgyaw+=other.yaw()
                    i+=1
        avgyaw/=i
        yyaw = (b.yaw() - avgyaw)/10
        b.cmdVelocityWorld(b.velocity(), yawRate=yyaw)

# boid simulation leader repulsion
def leaderRepulsion():
    global leaders
    for leader in leaders:
        force = np.array([0, 0, 0])
        for other in leaders:
            if other is not leader:
                dist = np.linalg.norm(leader.position() - other.position())
                if dist < 1:
                    force += (leader.position() - other.position()) / (dist*dist*dist)
        # normalize the velocity
        vel = leader.velocity() + force
        f = np.linalg.norm(vel)
        if f > 1:
            vel = vel / f
        leader.cmdVelocityWorld(vel, yawRate=0)



if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)

    # create a leader
    leaders.append(allcfs.crazyflies[0])
    leaders.append(allcfs.crazyflies[1])

    # run sim for 1min
    while timeHelper.time() < 60:
        repelling(allcfs.crazyflies)
        following(allcfs.crazyflies)
        leaderRepulsion()
        cohesion(allcfs.crazyflies)
        timeHelper.sleepForRate(sleepRate)