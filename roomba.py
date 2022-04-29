import gurobipy as gp
from gurobipy import GRB
from itertools import combinations
import matplotlib.pyplot as plt
from random import randrange
import json

grid_width = 1000
grid_height = 1000
roomba_width = 25
steps = 15

def randx():
    return randrange(roomba_width // 2, grid_width - roomba_width // 2)
def randy():
    return randrange(roomba_width // 2, grid_height - roomba_width // 2)


# Create a new model
m = gp.Model("Collision-Avoiding Robot Guidance")

class Roomba:
    def __init__(self, p_start, p_target, speed = 100):
        self.p_start = p_start
        self.p_target = p_target
        self.speed = speed

        self.pos_x_vars = [m.addVar(lb=0, ub=grid_width, vtype=GRB.CONTINUOUS) for _ in range(steps)]
        self.pos_y_vars = [m.addVar(lb=0, ub=grid_width, vtype=GRB.CONTINUOUS) for _ in range(steps)]
        self.mov_x_vars = [m.addVar(lb=-self.speed, ub=self.speed, vtype=GRB.CONTINUOUS) for _ in range(steps)]
        self.mov_y_vars = [m.addVar(lb=-self.speed, ub=self.speed, vtype=GRB.CONTINUOUS) for _ in range(steps)]

        # add start and end positions
        m.addConstr(self.pos_x_vars[0] == p_start[0])
        m.addConstr(self.pos_y_vars[0] == p_start[1])
        m.addConstr(self.pos_x_vars[-1] == p_target[0])
        m.addConstr(self.pos_y_vars[-1] == p_target[1])

        # add movement constraints
        for n in range(0, steps):
            # set maximum speed per step
            sqrspeed = self.speed**2
            m.addConstr(self.mov_x_vars[n]**2 + self.mov_y_vars[n]**2 <= sqrspeed)
            # add travelling constraint
            if n > 0: 
                m.addConstr(self.pos_x_vars[n] == self.pos_x_vars[n-1] + self.mov_x_vars[n-1])
                m.addConstr(self.pos_y_vars[n] == self.pos_y_vars[n-1] + self.mov_y_vars[n-1])

# Create Roombas
roombas = list()
for _ in range(20):
    start = (randx(), randy())
    target = (randx(), randy())
    roombas.append(Roomba(start, target))

# Collision-Avoidance Terms
for r1, r2 in combinations(roombas, 2):
    for n in range(steps):
        dx = r1.pos_x_vars[n] - r2.pos_x_vars[n]
        dy = r1.pos_y_vars[n] - r2.pos_y_vars[n]
        min_sqrdist = roomba_width**2
        sqrdist_term = dx**2 + dy**2
        m.addConstr(min_sqrdist <= sqrdist_term)

# Define Objective
objective = 0
for r in roombas:
    sqrspeed = r.speed**2
    objective += sum(r.mov_x_vars[i]**2 + r.mov_y_vars[i]**2 for i in range(steps))

m.setObjective(objective, GRB.MINIMIZE)
m.setParam('Timelimit', 60*5)
m.setParam('NonConvex', 2)
m.addGenConstrNorm()
m.optimize()

plt.clf()
paths = list()
for r in roombas:
    movv = [(r.mov_x_vars[i].x, r.mov_y_vars[i].x) for i in range(steps)]
    posv = [(r.pos_x_vars[i].x, r.pos_y_vars[i].x) for i in range(steps)]
    paths.append(posv)
    (x, y) = zip(*posv)
    # roomba size
    plt.scatter(x, y)

with open("paths.json", "w") as f:
        json.dump(paths, f)
plt.savefig("out.png")