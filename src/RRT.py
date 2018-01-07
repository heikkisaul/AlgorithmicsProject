"""
/   Goal-biased Rapidly-exploring Random Tree based path planning
/   Authors: Heikki Saul, Anders Martoja
/   Based on RRT by Steven M. LaValle
/   http://msl.cs.illinois.edu/~lavalle/sub/rrt.py
/   Open-source with GPL-3.0 licence
"""

import cv2
import random
import math
import time


STEPSIZE = 10
BIAS = 5
START = (50, 50)
GOAL = (950, 950)
MAXITER = 10000
WORLD = "Test3.png"


def dist(p1, p2):
    return math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))


def step_from_to(p1, p2, stepsize):
    if dist(p1, p2) < stepsize:
        return p2
    else:
        theta = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        return int(p1[0] + stepsize * math.cos(theta)), int(p1[1] + stepsize * math.sin(theta))


def bias_picker():
    bias = random.randint(1, 100)

    if bias <= BIAS:
        return 1
    else:
        return 0


def rapid_random_tree(array, start, goal, stepsize=10, width=1000, height=1000):
    random.seed()
    i = 0
    nodes = []
    tree_edges = []
    done = False
    q_r = start
    q_g = goal
    nodes.append(q_r)

    print("Percentage done of MAXITER:")

    while (not done) and i < MAXITER:

        if (1 + i) % (MAXITER/10) == 0:
            print(int(((1+i) / MAXITER)*100))

        random.seed()
        i += 1
        nn = nodes[0]

        # pick node to move towards
        bias = bias_picker()

        if bias == 1:
            q_rand = q_g
        elif bias == 0:
            q_rand = (int(random.random() * width), int(random.random() * height))

        for p in nodes:
            if dist(p, q_rand) < dist(nn, q_rand):
                nn = p

        new_node = step_from_to(nn, q_rand, stepsize)
        new_edge = nn, new_node

        if array[new_node[1]][new_node[0]] < 10:
            continue

        nodes.append(new_node)
        tree_edges.append(new_edge)

        if new_node == q_g:
            print("===========================")
            print("REACHED GOAL")
            print("No. or iterations: "+str(i))
            return tree_edges
    return tree_edges


if __name__ == "__main__":

    path = []
    path_len = 0

    timer_start = time.time()
    img = cv2.imread(WORLD, cv2.IMREAD_GRAYSCALE)
    edges = rapid_random_tree(img, START, GOAL, stepsize=STEPSIZE, width=1000, height=1000)

    e_end = (edges[-1])

    while e_end[0] != START:
        for e in edges:
            if e[1] == e_end[0]:
                path.append(e_end)
                e_end = e
        path.append(e_end)

    timer_end = time.time()
    timing = timer_end - timer_start

    print("Time elapsed: "+str(timing)+" seconds")

    img_c = cv2.imread(WORLD)

    for e in edges:
        cv2.line(img_c, e[0], e[1], (127, 127, 127), 1)

    for e in path:
        cv2.line(img_c, e[0], e[1], (0, 0, 255), 2)
        path_len += abs(dist(e[0], e[1]))

    print("Path length: "+str(int(path_len)//2))

    cv2.imshow('image', img_c)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
