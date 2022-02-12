import numpy as np
import math
from queue import Queue


def BFS(env_map, start_node, target_node):
    target = target_node
    start = start_node
    visited = set()
    queue = Queue()
    path = []

    queue.put(target)
    visited.add(start)

    parent = dict()
    parent[start_node] = None

    path_found = False
    while not queue.empty():
        current_node = queue.get()
        if current_node == target_node:
            path_found = True
            break

        neighbors_set = []
        y, x = current_node

        if (y + 1) <= 99:
            if env_map[y + 1, x] == 0:
                neighbors_set.append((y + 1, x))
        if (y - 1) >= 0:
            if env_map[y - 1, x] == 0:
                neighbors_set.append((y - 1, x))
        if (x + 1) <= 99:
            if env_map[y, x + 1] == 0:
                neighbors_set.append((y, x + 1))
        if (x - 1) >= 0:
            if env_map[y, x - 1] == 0:
                neighbors_set.append((y, x - 1))

        for next_node in neighbors_set:
            if next_node not in visited:
                queue.put(next_node)
                parent[next_node] = current_node
                visited.add(next_node)

    if path_found:
        path.append(target_node)
        while parent[target_node] is not None:
            path.append(parent[target_node])
            target_node = parent[target_node]
        path.reverse()

    return path


def moves_sequence(path, start_node, target_node):
    target = target_node
    start = start_node
    moveSequence = []
    step_counter = 0
    direction = 0
    last_step = 0

    for step in path:
        if step == start:
            direction = 0  # North
            change = 0  # No direction change
            last_step = step
            step_counter = 0
            continue
        if step == target:
            print("target node found")
            moveSequence.append((step_counter, 0, direction))
            if direction == 1:
                moveSequence.append((0, 2, 0))
            if direction == 2:
                moveSequence.append((0, 1, 0))
            continue

        y_current, x_current = step
        y_last, x_last = last_step

        if direction == 0 and x_current == x_last:
            step_counter += 1
            last_step = step
            continue
        if (direction == 1 or direction == 2) and y_current == y_last:
            step_counter += 1
            last_step = step
            continue
        if direction == 0 and x_current != x_last:
            moveSequence.append((step_counter, 0, direction))
            step_counter = 0
            if x_current > x_last:
                direction = 1
                moveSequence.append((step_counter, 1, direction))
                last_step = step
                continue
            else:
                direction = 2
                moveSequence.append((step_counter, 2, direction))
                last_step = step
                continue

        if (direction == 1 or direction == 2) and y_current != y_last:
            moveSequence.append((step_counter, 0, direction))
            step_counter = 0
            if direction == 1:
                direction = 0
                moveSequence.append((step_counter, 2, direction))
                last_step = step
                continue
            else:
                direction = 0
                moveSequence.append((step_counter, 1, direction))
                last_step = step
                continue

    item_num = len(moveSequence)
    step_counter = 0
    stop_index = 0

    if item_num > 2:
        for index in range(2, item_num):
            steps, change_direction, direction = moveSequence[index]
            if steps < 50:
                step_counter = step_counter + steps
                if change_direction == 0 and steps == 0:
                    step_counter = step_counter + 1
            if steps > 50:
                stop_index = index
                break
        steps_, change_, direction_ = moveSequence[2]
        moveSequence[2] = (step_counter, change_, direction_)
        del moveSequence[3:stop_index - 1]

    return moveSequence
