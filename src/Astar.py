# Author: Christian Careaga (christian.careaga7@gmail.com)
# A* Pathfinding in Python
# Please give credit if used
# Edited by Heikki Saul, Anders Martoja
# Open-source with GPL-3.0 licence

import cv2
import numpy as np
import time
from heapq import *
import math

WORLD = "Test2.png"
START = (50, 50)
GOAL = (950, 950)
STEPSIZE = 10

img = cv2.imread(WORLD)


def dist(p1, p2):
    return math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))


def heuristic(a, b, b_weight):
    if b_weight > 1:
        return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) * b_weight
    else:
        return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


def astar(array, startpoint, goal):
    global neighbor
    neighbors = []

    for x in range(-STEPSIZE, STEPSIZE + 1):
        for y in range(-STEPSIZE, STEPSIZE + 1):
            if x == STEPSIZE or x == -STEPSIZE or y == STEPSIZE or y == -STEPSIZE:
                neighbors.append((x, y))

    neighbors = sorted(list(set(neighbors)))

    goal_weight = 0

    close_set = set()
    came_from = {}
    gscore = {startpoint: 0}
    fscore = {startpoint: heuristic(startpoint, goal, goal_weight)}
    oheap = []

    heappush(oheap, (fscore[startpoint], startpoint))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal or (
                (goal[0] - STEPSIZE // 2) <= current[0] <= (goal[0] + STEPSIZE // 2) and (goal[1] - STEPSIZE // 2) <=
                current[1] <= (goal[1] + STEPSIZE // 2)):
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data, fscore[neighbor]

        close_set.add(current)
        for i_n, j in neighbors:
            neighbor = current[0] + i_n, current[1] + j
            neighbor_weight = nmap[neighbor[0]][neighbor[1]]
            img[neighbor[0]][neighbor[1]] = [200, 200, 200]
            tentative_g_score = gscore[current] + heuristic(current, neighbor, neighbor_weight)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                img[neighbor[0]][neighbor[1]] = [200, 200, 200]
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i_h[1] for i_h in oheap]:
                img[neighbor[0]][neighbor[1]] = [200, 200, 200]
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal, neighbor_weight)
                heappush(oheap, (fscore[neighbor], neighbor))

    img[neighbor[0]][neighbor[1]] = [200, 200, 200]
    return False


'''Here is an example of using my algo with a numpy array,
   astar(array, start, destination)
   astar function returns a list of points (shortest path)'''

if __name__ == "__main__":

    path_len = 0

    nmap = np.zeros((1000, 1000), np.uint8)

    for rcounter in range(1, 1000):
        for ccounter in range(1, 1000):
            if sum(img[rcounter][ccounter]) < 30:
                nmap[rcounter][ccounter] = 1

    start = time.time()
    path, length = astar(nmap, START, GOAL)
    end = time.time()
    timing = end - start
    print("Elapsed time: " + str(timing) + " seconds")
    for pixel in path:
        row = pixel[0]
        column = pixel[1]

        img[row][column] = [0, 0, 0]

    for i in range(len(path) - 1):
        y1 = path[i][0]
        x1 = path[i][1]
        y2 = path[i + 1][0]
        x2 = path[i + 1][1]
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        path_len += dist(path[i], path[i + 1])
    path_len += dist(path[-2], path[-1])
    path_len += dist(path[-1], START)
    cv2.line(img, (x1, y1), START, (0, 0, 255), 2)

    print("Path length: " + str(int(path_len)))

    for rcounter in range(1, 1000):
        for ccounter in range(1, 1000):
            if nmap[rcounter][ccounter] == 1:
                nmap[rcounter][ccounter] = 255

    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
