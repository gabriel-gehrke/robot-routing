import gurobipy as gp
from gurobipy import GRB
from itertools import combinations
import matplotlib.pyplot as plt
from random import randrange
from math import sqrt
import json

grid_width = 100
grid_height = 100
roomba_width = 10
steps = 20

def distance(p1, p2):
    return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def randx():
    return randrange(roomba_width // 2, grid_width - roomba_width // 2)
def randy():
    return randrange(roomba_width // 2, grid_height - roomba_width // 2)


# Create a new model
m = gp.Model("Collision-Avoiding Robot Guidance")

class Roomba:
    def __init__(self, p_start, p_target, speed = 10):
        self.p_start = p_start
        self.p_target = p_target
        self.speed = speed
        sqrspeed = self.speed**2

        self.pos_x_vars = [m.addVar(lb=0, ub=grid_width, vtype=GRB.CONTINUOUS) for _ in range(steps)]
        self.pos_y_vars = [m.addVar(lb=0, ub=grid_width, vtype=GRB.CONTINUOUS) for _ in range(steps)]
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
grid_points = [(x, y) for x in range(0, grid_width, roomba_width) for y in range(0, grid_height, roomba_width)]

for _ in range(5):
    start = grid_points.pop(randrange(0, len(grid_points)))
    target = grid_points.pop(randrange(0, len(grid_points)))
    roombas.append(Roomba(start, target))

# Collision-Avoidance Terms
for r1, r2 in combinations(roombas, 2):
    for n in range(steps):
        dx = r1.pos_x_vars[n] - r2.pos_x_vars[n]
        dy = r1.pos_y_vars[n] - r2.pos_y_vars[n]
        m.addConstr(roomba_width**2 <= dx**2 + dy**2)

# Define Objective
objective = 0
for r in roombas:
    sqrspeed = r.speed**2
    objective += sum(xmov**2 + ymov**2 for xmov, ymov in zip(r.mov_x_vars, r.mov_y_vars))

m.setObjective(objective, GRB.MINIMIZE)
m.setParam('Threads', 12)
m.setParam('NonConvex', 2)
m.optimize()

# get and plot paths
paths = list()
plt.clf()
plt.figure().set_size_inches(10, 10)
plt.axis([0, grid_width, 0, grid_height])
for r in roombas:
    movv = [(r.mov_x_vars[i].x, r.mov_y_vars[i].x) for i in range(steps - 1)]
    posv = [(r.pos_x_vars[i].x, r.pos_y_vars[i].x) for i in range(steps)]
    paths.append(posv)
    (x, y) = zip(*posv)
    # roomba size
    plt.scatter(x, y)

with open("paths.json", "w") as f:
        json.dump(paths, f)
plt.savefig("out.png")