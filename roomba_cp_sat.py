from itertools import combinations, chain
from random import randrange, random
from math import sqrt
from uuid import uuid4
from ortools.sat.python.cp_model import CpModel, CpSolver
import matplotlib.pyplot as plt
import json

"""
This is an "easier" formulation of the problem, which models the roombas as squares
and allows to make some optimizations.
"""

grid_width = 100
grid_height = 100
roomba_width = 16
half_roomba_width = roomba_width//2
steps = 15

# Create a new model
model = CpModel()
solver = CpSolver()

class Roomba:
    def __init__(self, p_start, p_target, speed = 10):
        id = uuid4()
        self.id = id
        self.p_start = p_start
        self.p_target = p_target
        self.speed = speed

        self.pos_x_vars = [model.NewIntVar(half_roomba_width, grid_width - half_roomba_width, f"{id}_pos_x_{i}") for i in range(steps)]
        self.pos_y_vars = [model.NewIntVar(half_roomba_width, grid_height - half_roomba_width, f"{id}_pos_y_{i}") for i in range(steps)]
        self.mov_x_vars = [model.NewIntVar(-speed, speed, f"{id}_mov_x_{i}") for i in range(steps - 1)]
        self.mov_y_vars = [model.NewIntVar(-speed, speed, f"{id}_mov_y_{i}") for i in range(steps - 1)]

        # add start and end positions
        model.Add(self.pos_x_vars[0] == p_start[0])
        model.Add(self.pos_y_vars[0] == p_start[1])
        model.Add(self.pos_x_vars[-1] == p_target[0])
        model.Add(self.pos_y_vars[-1] == p_target[1])

        # add movement constraints
        for n in range(1, steps):
            model.Add(self.pos_x_vars[n] == self.pos_x_vars[n-1] + self.mov_x_vars[n-1])
            model.Add(self.pos_y_vars[n] == self.pos_y_vars[n-1] + self.mov_y_vars[n-1])

    def extract_positions(self):
        return [(solver.Value(self.pos_x_vars[i]), solver.Value(self.pos_y_vars[i])) for i in range(steps)]
    
    def extract_movements(self):
        return [(solver.Value(self.mov_x_vars[i]), solver.Value(self.mov_y_vars[i])) for i in range(steps - 1)]

# Create Roombas
roombas = list()
grid_points = [(x, y) for x in range(half_roomba_width, grid_width, roomba_width) for y in range(half_roomba_width, grid_height, roomba_width)]

for _ in range(4):
    start = grid_points.pop(randrange(0, len(grid_points)))
    target = grid_points.pop(randrange(0, len(grid_points)))
    roombas.append(Roomba(start, target))

# Collision-Avoidance Terms
for n in range(1, steps - 1):
    x_intervals = list()
    y_intervals = list()

    for r in roombas:
        x = r.pos_x_vars[n]
        y = r.pos_y_vars[n]

        ix = model.NewFixedSizeIntervalVar(x, roomba_width, f"{r.id}_interval_x_step_{n}")
        iy = model.NewFixedSizeIntervalVar(y, roomba_width, f"{r.id}_interval_x_step_{n}")

        x_intervals.append(ix)
        y_intervals.append(iy)
    
    model.AddNoOverlap2D(x_intervals, y_intervals)

print(model.ModelStats())
status = solver.Solve(model)
print(solver.SolutionInfo())
print(solver.ResponseStats())
print()


# get and plot paths
paths = list()
plt.axis([0, grid_width, 0, grid_height])
figure, axes = plt.subplots()
axes.set_xlim((0, grid_width))
axes.set_ylim((0, grid_height))
axes.set_aspect(grid_width / grid_height)

for r in roombas:
    color = tuple(random() for _ in range(3))
    posv = r.extract_positions()
    movv = r.extract_movements()
    assert posv[0] == r.p_start
    assert posv[-1] == r.p_target

    for r2 in roombas:
        if r == r2: continue
        for ((x1, y1), (x2, y2)) in zip(posv, r2.extract_positions()):
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