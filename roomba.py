import gurobipy as gp
from gurobipy import GRB
from itertools import combinations, chain
import matplotlib.pyplot as plt
from random import randrange, random
from math import sqrt
import json

grid_width = 100
grid_height = 100
roomba_width = 16
half_roomba_width = roomba_width//2
steps = 15

def distance(p1, p2):
    return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


# Create a new model
m = gp.Model("Collision-Avoiding Robot Guidance")

class Roomba:
    def __init__(self, p_start, p_target, speed = 10):
        self.p_start = p_start
        self.p_target = p_target
        self.speed = speed
        sqrspeed = self.speed**2

        self.pos_x_vars = [m.addVar(lb=half_roomba_width, ub=grid_width - half_roomba_width, vtype=GRB.CONTINUOUS) for _ in range(steps)]
        self.pos_y_vars = [m.addVar(lb=half_roomba_width, ub=grid_width - half_roomba_width, vtype=GRB.CONTINUOUS) for _ in range(steps)]
        self.mov_x_vars = [m.addVar(lb=-self.speed, ub=self.speed, vtype=GRB.CONTINUOUS) for _ in range(steps - 1)]
        self.mov_y_vars = [m.addVar(lb=-self.speed, ub=self.speed, vtype=GRB.CONTINUOUS) for _ in range(steps - 1)]

        # add start and end positions
        m.addConstr(self.pos_x_vars[0] == p_start[0])
        m.addConstr(self.pos_y_vars[0] == p_start[1])
        m.addConstr(self.pos_x_vars[-1] == p_target[0])
        m.addConstr(self.pos_y_vars[-1] == p_target[1])

        # limit movement vector:
        for xmov, ymov in zip(self.mov_x_vars, self.mov_y_vars):
            m.addConstr(xmov**2 + ymov**2 <= sqrspeed)

        # add movement constraints
        for n in range(1, steps):
            m.addConstr(self.pos_x_vars[n] == self.pos_x_vars[n-1] + self.mov_x_vars[n-1])
            m.addConstr(self.pos_y_vars[n] == self.pos_y_vars[n-1] + self.mov_y_vars[n-1])

# Create Roombas
roombas = list()
grid_points = [(x, y) for x in range(half_roomba_width, grid_width, roomba_width) for y in range(half_roomba_width, grid_height, roomba_width)]

for _ in range(7):
    start = grid_points.pop(randrange(0, len(grid_points)))
    target = grid_points.pop(randrange(0, len(grid_points)))
    roombas.append(Roomba(start, target))

# Collision-Avoidance Terms
for r1, r2 in combinations(roombas, 2):
    for n in range(steps):
        dx = r1.pos_x_vars[n] - r2.pos_x_vars[n]
        dy = r1.pos_y_vars[n] - r2.pos_y_vars[n]
        min_sqr_dist = roomba_width**2
        m.addConstr(dx**2 + dy**2 >= min_sqr_dist)

# Define Objective
#objective = 0
#for r in roombas:
#    sqrspeed = r.speed**2
#    objective += sum(xmov**2 + ymov**2 for xmov, ymov in zip(r.mov_x_vars, r.mov_y_vars))
#m.setObjective(objective, GRB.MINIMIZE)

m.setParam('Threads', 12)
m.setParam('NonConvex', 2)
m.update()
m.optimize()


# get and plot paths
paths = list()
plt.axis([0, grid_width, 0, grid_height])
figure, axes = plt.subplots()
axes.set_xlim((0, grid_width))
axes.set_ylim((0, grid_height))
axes.set_aspect(grid_width / grid_height)

circles = list()
arrows = list()
for r in roombas:
    color = tuple(random() for _ in range(3))
    movv = [(r.mov_x_vars[i].x, r.mov_y_vars[i].x) for i in range(steps - 1)]
    posv = [(r.pos_x_vars[i].x, r.pos_y_vars[i].x) for i in range(steps)]

    for i, (x, y) in enumerate(posv):
        # circle
        circles.append(plt.Circle((x, y), radius=half_roomba_width, color=color, fill=False))
        # arrow
        if i < steps - 1:
            (dx, dy) = movv[i]
            arrows.append(plt.arrow(x, y, dx, dy, width=0.1, head_width=2, color=color, shape="full"))

    paths.append(posv)

for artist in chain(circles, arrows):
    axes.add_artist(artist)

with open("paths.json", "w") as f:
        json.dump(paths, f)
plt.savefig("out.png")