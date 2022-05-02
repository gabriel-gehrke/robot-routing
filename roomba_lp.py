import gurobipy as gp
from gurobipy import GRB
from itertools import combinations, chain
import matplotlib.pyplot as plt
from random import randrange, random
from math import sqrt
import json

"""
This is an "easier" formulation of the problem, which models the roombas as squares
and allows to make some optimizations.
"""

grid_width = 100
grid_height = 100
roomba_width = 16
half_roomba_width = roomba_width//2
steps = 30

# Create a new model
m = gp.Model("Collision-Avoiding Robot Guidance")

class Roomba:
    def __init__(self, p_start, p_target, speed = 10.0):
        self.p_start = p_start
        self.p_target = p_target
        self.speed = speed

        self.pos_x_vars = m.addVars(steps, lb = float(half_roomba_width), ub = float(grid_width - half_roomba_width))
        self.pos_y_vars = m.addVars(steps, lb = float(half_roomba_width), ub = float(grid_height - half_roomba_width))
        self.mov_x_vars = m.addVars(steps - 1, lb = -speed, ub = speed)
        self.mov_y_vars = m.addVars(steps - 1, lb = -speed, ub = speed)

        # add start and end positions
        m.addConstr(self.pos_x_vars[0] == float(p_start[0]))
        m.addConstr(self.pos_y_vars[0] == float(p_start[1]))
        m.addConstr(self.pos_x_vars[steps - 1] == float(p_target[0]))
        m.addConstr(self.pos_y_vars[steps - 1] == float(p_target[1]))

        # add movement constraints
        for n in range(1, steps):
            m.addConstr(self.pos_x_vars[n] == self.pos_x_vars[n-1] + self.mov_x_vars[n-1])
            m.addConstr(self.pos_y_vars[n] == self.pos_y_vars[n-1] + self.mov_y_vars[n-1])

    def extract_positions(self):
        return [(round(self.pos_x_vars[i].x, 2), round(self.pos_y_vars[i].x, 2)) for i in range(steps)]
    
    def extract_movements(self):
        return [(self.mov_x_vars[i].x, self.mov_y_vars[i].x) for i in range(steps - 1)]

# Create Roombas
roombas = list()
grid_points = [(x, y) for x in range(half_roomba_width, grid_width, roomba_width) for y in range(half_roomba_width, grid_height, roomba_width)]

for _ in range(14):
    start = grid_points.pop(randrange(0, len(grid_points)))
    target = grid_points.pop(randrange(0, len(grid_points)))
    roombas.append(Roomba(start, target))

# Collision-Avoidance Terms
for r1, r2 in combinations(roombas, 2):
    for n in range(1, steps - 1):
        # model distance along x and y axis
        x1 = r1.pos_x_vars[n]
        x2 = r2.pos_x_vars[n]
        y1 = r1.pos_y_vars[n]
        y2 = r2.pos_y_vars[n]
        dx = x1 - x2
        dy = y1 - y2

        # dummy variables (binary)
        a = m.addVar(vtype = GRB.BINARY)
        b = m.addVar(vtype = GRB.BINARY)
        c = m.addVar(vtype = GRB.BINARY)
        d = m.addVar(vtype = GRB.BINARY)

        # x distance
        m.addConstr((a == 1) >> (dx <= -roomba_width))
        m.addConstr((b == 1) >> (dx >= roomba_width))

        # y distance
        m.addConstr((c == 1) >> (dy <= -roomba_width))
        m.addConstr((d == 1) >> (dy >= roomba_width))

        # min distance must be given in at least one direction
        m.addConstr(a + b + c + d >= 1)

m.setParam('Threads', 12)
#m.setParam('NonConvex', 2)
m.update()


m.optimize()
if m.status == GRB.INFEASIBLE:
    print("Infeasible!")
    exit(1)


# get and plot paths
paths = list()
plt.axis([0, grid_width, 0, grid_height])
figure, axes = plt.subplots()
axes.set_xlim((0, grid_width))
axes.set_ylim((0, grid_height))
axes.set_aspect(grid_width / grid_height)

for r in roombas:
    color = tuple(random() for _ in range(3))
    movv = r.extract_movements()
    posv = r.extract_positions()
    assert posv[0] == r.p_start
    assert posv[-1] == r.p_target

    # no-collision assertions
    for r2 in roombas:
        if r == r2: continue
        for ((x1, y1), (x2, y2)) in zip(posv, r2.extract_positions()):
            #print(f"dx = {abs(x1 - x2)}, dy = {abs(y1 - y2)}")
            assert abs(x1 - x2) >= roomba_width or abs(y1 - y2) >= roomba_width

    for i, (x, y) in enumerate(posv):
        # rect
        axes.add_artist(plt.Rectangle((x - half_roomba_width, y - half_roomba_width), roomba_width, roomba_width, color=color, fill=False))
        # arrow
        if i < steps - 1:
            (dx, dy) = movv[i]
            plt.arrow(x, y, dx, dy, width=0.1, head_width=2, color=color, shape="full", length_includes_head=True)

    paths.append(posv)

with open("paths.json", "w") as f:
        json.dump(paths, f)
plt.savefig("out.png")